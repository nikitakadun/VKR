"""
SAHR Controller - Optimized for Stable Hopping
Reduced speeds and torques within hardware limits
"""

from controller import Robot, Motor, PositionSensor, InertialUnit, GPS, TouchSensor, DistanceSensor
import math

# ==================== КОНСТАНТЫ (В ПРЕДЕЛАХ ОГРАНИЧЕНИЙ) ====================
TIME_STEP = 32  # ms
SIMULATION_TIME = 30  # seconds

# Параметры управления (Level 3 - целевые параметры)
DESIRED_HEIGHT = 0.30  # желаемая высота прыжка (м)
DESIRED_VERTICAL_VELOCITY = 2.5  # желаемая вертикальная скорость (м/с)

# Коэффициенты корректора (Level 3)
K_V = 0.8  
K_H = 1.2  

# Параметры PID для центробежного мотора (Level 1) - уменьшены
Kp = 12.0   
Ki = 3.0    
Kd = 0.8    

# Параметры управления ногой
HIP_ANGLE_COMPRESS = 0.15     # угол при сжатии пружины
HIP_ANGLE_JUMP = 0.4          # угол в момент отрыва
HIP_ANGLE_FLIGHT = 0.25       # угол в полёте
HIP_ANGLE_LANDING = -0.1      # угол при приземлении
HIP_ANGLE_PREPARE = 0.0       # подготовка

# Параметры автомата фаз (уменьшенные скорости)
CONTACT_THRESHOLD = 0.002     
FOOT_FORCE_THRESHOLD = 0.03   
MAX_COMPRESSION = 0.008       

# Параметры энергетического цикла - В ПРЕДЕЛАХ ЛИМИТОВ
SPIN_UP_DURATION = 0.5        
SPIN_UP_SPEED = 10.0          # было 60, теперь 45 (maxVelocity=60, запас)
JUMP_BOOST_DURATION = 0.08    
JUMP_BOOST_POWER = 15.0       # было 50, теперь 25 (maxTorque=100)

# Ограничения моторов (из world файла)
MAX_HIP_VELOCITY = 30.0       
MAX_CENTRIFUGAL_VELOCITY = 60.0   # жесткий лимит из world
MAX_CENTRIFUGAL_TORQUE = 100.0    # жесткий лимит из world

# Преобразователь угол -> скорость
K_PHI = 6.0  


class Phase:
    START = 0
    SPIN_UP = 1
    JUMP = 2
    FLIGHT = 3
    LANDING = 4
    RECOVERY = 5
    
    @staticmethod
    def get_name(phase):
        names = {0: "START", 1: "SPIN_UP", 2: "JUMP", 3: "FLIGHT", 4: "LANDING", 5: "RECOVERY"}
        return names.get(phase, "UNKNOWN")


class SAHRController:
    def __init__(self, robot):
        self.robot = robot
        self.time_step = TIME_STEP
        self.current_time = 0.0
        self.jump_count = 0
        
        # ========== ИНИЦИАЛИЗАЦИЯ УСТРОЙСТВ ==========
        self.hip_motor = robot.getDevice("hip_motor")
        self.centrifugal_motor = robot.getDevice("centrifugal_motor")
        
        self.hip_sensor = robot.getDevice("hip_sensor")
        self.centrifugal_sensor = robot.getDevice("centrifugal_sensor")
        self.leg_sensor = robot.getDevice("leg_sensor")
        self.foot_sensor = robot.getDevice("foot_sensor")
        self.imu = robot.getDevice("inertial_unit")
        self.gps = robot.getDevice("body_gps")
        self.height_sensor = robot.getDevice("height_sensor")
        
        # Включение датчиков
        self.hip_sensor.enable(TIME_STEP)
        self.centrifugal_sensor.enable(TIME_STEP)
        self.leg_sensor.enable(TIME_STEP)
        self.foot_sensor.enable(TIME_STEP)
        self.imu.enable(TIME_STEP)
        self.gps.enable(TIME_STEP)
        self.height_sensor.enable(TIME_STEP)
        
        # ========== ПЕРЕМЕННЫЕ ==========
        self.phase = Phase.START
        self.phase_timer = 0.0
        
        # PID переменные
        self.integral_error = 0.0
        self.prev_error = 0.0
        
        # Оценка состояния
        self.estimated_height = 0.0
        self.estimated_vertical_velocity = 0.0
        self.prev_height = 0.0
        
        # Управление
        self.desired_height = DESIRED_HEIGHT
        self.desired_velocity = DESIRED_VERTICAL_VELOCITY
        self.omega_desired = 0.0
        self.phi_desired = 0.0
        
        # Состояние
        self.contact = False
        self.leg_compression = 0.0
        self.stored_energy = 0.0
        self.prev_omega = 0.0
        
        self.last_log_time = 0.0
        self.warning_shown = False
        
        print("=== SAHR Controller Optimized ===")
        print(f"Max motor speed: {MAX_CENTRIFUGAL_VELOCITY} rad/s")
        print(f"Max motor torque: {MAX_CENTRIFUGAL_TORQUE} Nm")
        print(f"Target speed: {SPIN_UP_SPEED} rad/s")
        
    def clamp(self, value, min_val, max_val):
        if value < min_val:
            return min_val
        if value > max_val:
            return max_val
        return value
    
    def detect_contact(self):
        """Детекция контакта"""
        self.leg_compression = abs(self.leg_sensor.getValue())
        foot_force = self.foot_sensor.getValue()
        
        k_spring = 50000.0
        self.stored_energy = 0.5 * k_spring * (self.leg_compression ** 2)
        
        contact = (self.leg_compression > CONTACT_THRESHOLD) or (foot_force > FOOT_FORCE_THRESHOLD)
        
        if contact and not self.contact:
            print(f"  >>> CONTACT at t={self.current_time:.3f}s | Energy: {self.stored_energy:.2f}J")
        elif not contact and self.contact:
            self.jump_count += 1
            print(f"  >>> LIFTOFF #{self.jump_count} at t={self.current_time:.3f}s | Height: {self.estimated_height:.3f}m")
            
        return contact
    
    def estimate_state(self):
        """Оценка высоты и скорости"""
        gps_values = self.gps.getValues()
        self.estimated_height = gps_values[2]
        
        dt = TIME_STEP / 1000.0
        if dt > 0:
            self.estimated_vertical_velocity = (self.estimated_height - self.prev_height) / dt
        
        self.prev_height = self.estimated_height
        return self.estimated_height, self.estimated_vertical_velocity
    
    def compute_errors(self):
        """Вычисление ошибок"""
        e_v = self.desired_velocity - self.estimated_vertical_velocity
        e_h = self.desired_height - self.estimated_height
        return e_v, e_h
    
    def corrector(self, e_v, e_h):
        """Корректор"""
        delta_omega = K_V * e_v + K_H * e_h
        delta_omega = self.clamp(delta_omega, -15.0, 15.0)
        return delta_omega, 0.0
    
    def pid_controller(self, desired_omega, actual_omega, dt):
        """ПИД-регулятор с ограничениями"""
        error = desired_omega - actual_omega
        
        P = Kp * error
        
        self.integral_error += error * dt
        self.integral_error = self.clamp(self.integral_error, -8.0, 8.0)
        I = Ki * self.integral_error
        
        if dt > 0:
            derivative = (error - self.prev_error) / dt
        else:
            derivative = 0.0
        D = Kd * derivative
        
        output = P + I + D
        
        # Жесткое ограничение в пределах maxTorque
        output = self.clamp(output, -MAX_CENTRIFUGAL_TORQUE, MAX_CENTRIFUGAL_TORQUE)
        
        self.prev_error = error
        return output
    
    def angle_to_velocity(self, desired_phi, actual_phi):
        """Преобразователь угол -> скорость"""
        error = desired_phi - actual_phi
        # Нелинейное преобразование для плавности
        if abs(error) > 0.5:
            gain = K_PHI * 1.2
        else:
            gain = K_PHI
            
        omega = gain * error
        return self.clamp(omega, -MAX_HIP_VELOCITY, MAX_HIP_VELOCITY)
    
    def phase_automaton(self):
        """Автомат фаз"""
        dt = TIME_STEP / 1000.0
        self.phase_timer += dt
        
        contact = self.contact
        centrifugal_speed = abs(self.centrifugal_sensor.getValue())
        
        if self.phase == Phase.START:
            if centrifugal_speed > SPIN_UP_SPEED * 0.6:
                self.change_phase(Phase.SPIN_UP)
                
        elif self.phase == Phase.SPIN_UP:
            if centrifugal_speed >= SPIN_UP_SPEED and self.leg_compression > CONTACT_THRESHOLD:
                self.change_phase(Phase.JUMP)
                
        elif self.phase == Phase.JUMP:
            if not contact or self.phase_timer > JUMP_BOOST_DURATION:
                self.change_phase(Phase.FLIGHT)
                
        elif self.phase == Phase.FLIGHT:
            if contact:
                self.change_phase(Phase.LANDING)
                
        elif self.phase == Phase.LANDING:
            if self.leg_compression > MAX_COMPRESSION * 0.7 or self.phase_timer > 0.12:
                self.change_phase(Phase.RECOVERY)
                
        elif self.phase == Phase.RECOVERY:
            if self.phase_timer > SPIN_UP_DURATION * 0.4:
                self.change_phase(Phase.SPIN_UP)
        
        self.phase_control()
    
    def change_phase(self, new_phase):
        """Смена фазы"""
        if self.phase != new_phase:
            print(f"[{self.current_time:.3f}s] {Phase.get_name(self.phase)} -> {Phase.get_name(new_phase)}")
            self.phase = new_phase
            self.phase_timer = 0.0
    
    def phase_control(self):
        """Управление в зависимости от фазы"""
        actual_phi = self.hip_sensor.getValue()
        actual_omega = self.centrifugal_sensor.getValue()
        
        if self.phase == Phase.START:
            progress = self.clamp(self.phase_timer / 0.2, 0.0, 1.0)
            self.omega_desired = SPIN_UP_SPEED * progress
            self.phi_desired = HIP_ANGLE_PREPARE
            
        elif self.phase == Phase.SPIN_UP:
            self.omega_desired = SPIN_UP_SPEED
            if self.leg_compression < MAX_COMPRESSION * 0.5:
                self.phi_desired = HIP_ANGLE_COMPRESS
            else:
                self.phi_desired = HIP_ANGLE_PREPARE
                
        elif self.phase == Phase.JUMP:
            e_v, e_h = self.compute_errors()
            delta_omega, _ = self.corrector(e_v, e_h)
            
            # Плавный буст
            boost = JUMP_BOOST_POWER * (1.0 - self.phase_timer / JUMP_BOOST_DURATION)
            boost = self.clamp(boost, 0.0, JUMP_BOOST_POWER)
            
            self.omega_desired = SPIN_UP_SPEED + delta_omega + boost
            # Жесткое ограничение скорости
            self.omega_desired = self.clamp(self.omega_desired, 0, MAX_CENTRIFUGAL_VELOCITY)
            
            progress = self.clamp(self.phase_timer / JUMP_BOOST_DURATION, 0.0, 1.0)
            self.phi_desired = HIP_ANGLE_COMPRESS + (HIP_ANGLE_JUMP - HIP_ANGLE_COMPRESS) * progress
            
        elif self.phase == Phase.FLIGHT:
            self.omega_desired = SPIN_UP_SPEED * 0.65
            
            landing_prepare = self.clamp(self.phase_timer / 0.25, 0.0, 1.0)
            self.phi_desired = HIP_ANGLE_FLIGHT + (HIP_ANGLE_LANDING - HIP_ANGLE_FLIGHT) * landing_prepare
            
        elif self.phase == Phase.LANDING:
            self.omega_desired = SPIN_UP_SPEED * 0.5
            
            if self.leg_compression < MAX_COMPRESSION * 0.3:
                self.phi_desired = HIP_ANGLE_LANDING
            else:
                self.phi_desired = HIP_ANGLE_COMPRESS
                
        elif self.phase == Phase.RECOVERY:
            self.omega_desired = SPIN_UP_SPEED
            self.phi_desired = HIP_ANGLE_COMPRESS
        
        # Применяем управление с ограничениями
        self.apply_control(actual_phi, actual_omega)
    
    def apply_control(self, actual_phi, actual_omega):
        """Применение управляющих сигналов"""
        dt = TIME_STEP / 1000.0
        
        # Управление ногой
        hip_velocity = self.angle_to_velocity(self.phi_desired, actual_phi)
        self.hip_motor.setVelocity(hip_velocity)
        
        # Управление центробежным мотором
        torque = self.pid_controller(self.omega_desired, actual_omega, dt)
        
        # Дополнительная проверка на превышение лимитов
        if abs(torque) > MAX_CENTRIFUGAL_TORQUE:
            torque = MAX_CENTRIFUGAL_TORQUE if torque > 0 else -MAX_CENTRIFUGAL_TORQUE
            
        self.centrifugal_motor.setTorque(torque)
        
        # Ограничиваем скорость, чтобы не было варнингов
        if self.omega_desired > MAX_CENTRIFUGAL_VELOCITY:
            self.centrifugal_motor.setVelocity(MAX_CENTRIFUGAL_VELOCITY)
        else:
            self.centrifugal_motor.setVelocity(self.omega_desired + 5.0)  # небольшой запас
    
    def log_state(self):
        """Логирование состояния"""
        if self.current_time - self.last_log_time >= 0.5:
            self.last_log_time = self.current_time
            
            centrifugal_speed = self.centrifugal_sensor.getValue()
            hip_angle = self.hip_sensor.getValue()
            leg_compression_mm = abs(self.leg_sensor.getValue()) * 1000
            
            # Проверка на превышение лимитов (только если превышает)
            if centrifugal_speed > MAX_CENTRIFUGAL_VELOCITY and not self.warning_shown:
                print(f"  WARNING: Speed {centrifugal_speed:.1f} > {MAX_CENTRIFUGAL_VELOCITY}")
                self.warning_shown = True
            
            speed_indicator = ""
            if abs(self.estimated_vertical_velocity) > 1.0:
                speed_indicator = " ↑" if self.estimated_vertical_velocity > 0 else " ↓"
            
            print(f"{self.current_time:5.2f}s | {Phase.get_name(self.phase):8s} | "
                  f"H:{self.estimated_height:5.3f}m V:{self.estimated_vertical_velocity:5.2f}m/s{speed_indicator} | "
                  f"ω:{centrifugal_speed:5.1f}/{self.omega_desired:5.1f} | "
                  f"Spring:{leg_compression_mm:4.1f}mm")
    
    def run(self):
        """Основной цикл"""
        print("Starting control loop...")
        print("=" * 70)
        
        self.hip_motor.setPosition(float('inf'))
        self.centrifugal_motor.setPosition(float('inf'))
        
        while self.robot.step(TIME_STEP) != -1:
            self.current_time += TIME_STEP / 1000.0
            
            if self.current_time >= SIMULATION_TIME:
                print(f"\n=== Finished: {self.jump_count} jumps in {SIMULATION_TIME}s ===")
                break
            
            self.contact = self.detect_contact()
            self.estimate_state()
            self.phase_automaton()
            self.log_state()
        
        print("Controller stopped")


if __name__ == "__main__":
    robot = Robot()
    controller = SAHRController(robot)
    controller.run()