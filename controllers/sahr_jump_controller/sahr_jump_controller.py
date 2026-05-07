"""SAHR Controller – один оборот груза → сжатие пружины → импульсный прыжок."""

from controller import Robot
import math
import csv
import os

TIME_STEP = 1                 # мс
NOMINAL_LEG_LENGTH = 0.1      # длина ноги без сжатия, м

# Фазы
(START, SPINUP, BRAKE_JUMP, FLIGHT, PRELAND, LANDING) = range(6)
PHASE_NAMES = ['START', 'SPINUP', 'BRAKE_JUMP', 'FLIGHT', 'PRELAND', 'LANDING']

# ---------- Параметры модели (из сцены Webots) ----------
M_BODY          = 10.0         # масса корпуса (Physics mass = 10), кг
M_GR            = 2.0          # масса центробежного груза (Physics mass = 2), кг
M_TOTAL         = M_BODY + M_GR  # общая масса корпуса с грузом, кг
K_SPRING        = 50000.0      # жёсткость пружины (springConstant = 50000), Н/м
R_GR            = 0.08         # радиус вращения груза (translation 0 0 0.08), м
I_GR            = M_GR * R_GR**2  # момент инерции груза = 2 * 0.0064 = 0.0128 кг·м²
R_ARM           = 0.5          # радиус рычага (оценка по геометрии стойки), м
R_PLECHO        = 0.05         # плечо передачи импульса (оценка), м
ETA             = 0.9          # КПД импульсной передачи
G               = 9.81
MAX_SPRING_COMP = 0.03         # максимальное сжатие пружины (ограничение), м
B_DAMPING       = 0.0          # коэффициент демпфирования пружины, Н·с/м (пока 0)

# ---------- Параметры регуляторов ----------
BASE_HIP_ANGLE   = -0.2619     # -15° (угол по умолчанию)
JUMP_HIP_ANGLE   = -0.35       # угол ноги во время сжатия/прыжка (носок вперёд)

FORWARD_HIP_ANGLE = 0.2619     # Наклон вперед на 15 градусов
BACKWARD_HIP_ANGLE = -0.2619

KP_SPIN          = 5.0         # П‑регулятор скорости груза
KP_HIP           = 400.0        # ПД‑регулятор угла бедра
KD_HIP           = 12.0
GR_TORQUE_MAX    = 100.0       # макс. момент груза (совпадает с maxTorque в сцене), Н·м
HIP_TORQUE_MAX   = 100.0        # макс. момент бедра (совпадает с maxTorque в сцене), Н·м
BRAKE_TORQUE     = -100.0      # тормозной момент (равен -GR_TORQUE_MAX)
BRAKE_DURATION   = 0.03        # длительность торможения, с
LANDING_HOLD     = 0.3         # время удержания после касания, с
FULL_TURN_RAD    = 2 * math.pi
ANGLE_TOLERANCE  = 0.05        # допуск при определении завершения оборота, рад

# ---------- Целевые показатели ----------
TARGET_HEIGHT  = 0.10          # желаемая высота прыжка, м
TARGET_SPEED_X = 0.5           # желаемая тангенциальная скорость, м/с

# ---------- Коэффициенты верхнего уровня ----------
KP_HEIGHT_ERR    = 20.0        # добавочная скорость груза на метр ошибки по высоте
KP_SPEED_ERR     = 0.2 

class SAHRController:
    def __init__(self):
        self.robot = Robot()

        # Датчики
        self.leg_sensor = self.robot.getDevice("leg_sensor")
        self.leg_sensor.enable(TIME_STEP)
        self.foot_sensor = self.robot.getDevice("foot_sensor")
        self.foot_sensor.enable(TIME_STEP)
        self.body_gps = self.robot.getDevice("body_gps")
        self.body_gps.enable(TIME_STEP)
        self.hip_sensor = self.robot.getDevice("hip_sensor")
        self.hip_sensor.enable(TIME_STEP)
        self.centrifugal_sensor = self.robot.getDevice("centrifugal_sensor")
        self.centrifugal_sensor.enable(TIME_STEP)

        # Моторы
        self.hip_motor = self.robot.getDevice("hip_motor")
        self.hip_motor.setTorque(0.0)
        self.centrifugal_motor = self.robot.getDevice("centrifugal_motor")
        self.centrifugal_motor.setTorque(0.0)

        # Переменные состояния
        self.phase = START
        self.prev_contact = False
        self.omega = 0.0
        self.prev_cent_angle = 0.0
        self.start_angle = None
        self.revolution_complete = False
        self.ground_z = None
        self.apex_height = 0.0
        self.apex_reached = False
        self.apex_time = 0.0
        self.prev_x = 0.0
        self.vx = 0.0
        self.vy = 0.0
        self.phase_start_time = 0.0
        self.brake_start_time = None
        self.prev_hip_error = 0.0
        self.prev_z = None

        # Переменные для SAHR-кинематики
        self.prev_leg_len = NOMINAL_LEG_LENGTH
        self.leg_velocity = 0.0          # r˙ – скорость изменения длины ноги
        self.hip_angular_velocity = 0.0  # φ˙ – угловая скорость бедра
        self.prev_hip_angle = 0.0
        self.vx_stance = 0.0             # x˙ в фазе опоры
        self.vy_stance = 0.0             # y˙ в фазе опоры
                

        # Целевые уставки
        self.target_spin_speed = 0.0
        self.target_hip_angle = -0.244346 # JUMP_HIP_ANGLE

        self.target_delta_r = 0

        # Создание CSV-файла для записи данных
        self.csv_file = open("sahr_data.csv", "w", newline="")
        self.csv_writer = csv.writer(self.csv_file)
        
        # Заголовки столбцов
        self.csv_writer.writerow([
            "time", "phase", 
            "z", "x", 
            "compression", "leg_length",
            "hip_angle", "target_hip_angle",
            "vx", "vy", 
            "omega", "contact"
        ])

        # Резонансная частота
        self.omega_res = math.sqrt(K_SPRING / M_BODY)
        print(f"Resonant frequency: {self.omega_res:.1f} rad/s")
        print("SAHR Controller (one-turn impulse) ready.")

    # ========== Сенсоры ==========
    def read_sensors(self):
        leg_off = self.leg_sensor.getValue() if self.leg_sensor else 0.0
        leg_len = NOMINAL_LEG_LENGTH + leg_off
        compression = NOMINAL_LEG_LENGTH - leg_len  # Δr = L - r (положительно при сжатии)
        gps = self.body_gps.getValues() if self.body_gps else [0,0,0]
        if self.prev_z == None:
            self.prev_z = gps[2]
        contact = self.foot_sensor.getValue() > 0.5 if self.foot_sensor else False
        cent_angle = self.centrifugal_sensor.getValue() if self.centrifugal_sensor else 0.0
        hip_angle = self.hip_sensor.getValue() if self.hip_sensor else 0.0
        return {'x': gps[0], 'z': gps[2], 'contact': contact, 'leg_len': leg_len,
                'compression': compression, 'cent_angle': cent_angle, 'hip_angle': hip_angle}

    # ========== Вычисление угловой скорости груза ==========
    def compute_omega(self, s):
        dt = TIME_STEP / 1000.0
        d = s['cent_angle'] - self.prev_cent_angle
        if d > math.pi:
            d -= 2 * math.pi
        elif d < -math.pi:
            d += 2 * math.pi
        self.omega = d / dt
        self.prev_cent_angle = s['cent_angle']

    # ========== Вычисление скоростей по SAHR-кинематике ==========
    def compute_stance_kinematics(self, s):
        dt = TIME_STEP / 1000.0
        self.leg_velocity = (s['leg_len'] - self.prev_leg_len) / dt
        self.prev_leg_len = s['leg_len']

        d_hip = s['hip_angle'] - self.prev_hip_angle
        if d_hip > math.pi:
            d_hip -= 2 * math.pi
        elif d_hip < -math.pi:
            d_hip += 2 * math.pi
        self.hip_angular_velocity = d_hip / dt
        self.prev_hip_angle = s['hip_angle']

    def compute_stance_velocity(self, s):
        """
        Уравнения (7) из статьи SAHR:
        x˙ = −r˙ sin φ − r φ˙ cos φ
        y˙ =  r˙ cos φ − r φ˙ sin φ
        """
        r = s['compression']
        phi = s['hip_angle']
        r_dot = self.leg_velocity
        phi_dot = self.hip_angular_velocity

        self.vx = -r_dot * math.sin(phi) - r * phi_dot * math.cos(phi)
        self.vy =  r_dot * math.cos(phi) - r * phi_dot * math.sin(phi)

    # ========== Горизонтальная скорость (GPS) ==========
    def compute_velocity(self, s):
        dt = TIME_STEP / 1000.0
        self.vx = (s['x'] - self.prev_x) / dt
        self.prev_x = s['x']

    # ========== Расчёт угла приземления ==========
    def compute_touchdown_angle(self):
 
    # Если скорость слишком мала — используем базовый угол
        if abs(self.vx) < 0.01 and abs(self.vy) < 0.01:
            return BASE_HIP_ANGLE
        
        # Угол вектора скорости относительно вертикали
        # vy < 0 при падении (робот снижается)
        if self.vy < 0:
            angle = -math.atan2(self.vx, -self.vy)
            return angle
        else:
            # Если vy положительна (взлетает) — используем базовый угол
            angle = BASE_HIP_ANGLE
        
        # Ограничиваем угол в разумных пределах (-34° до +15°)
        return max(-0.6, min(0.2619, angle))
        
    # ========== Расчёт угла прыжка ==========
    def compute_jump_angle(self):
        pass
    
    def compute_vy_no_contact(self, s):
        dt = TIME_STEP / 1000.0
        self.vy = (s['z'] - self.prev_z ) / dt
        self.prev_z = s['z']
        
    # ========== Расчёт уставок по желаемой высоте ==========
    def calculate_setpoints(self, target_height):
        v_to = math.sqrt(2 * G * target_height)
        E_jump = 0.5 * M_BODY * v_to ** 2

        # Энергия пружины при максимальном сжатии
        delta_r_max = MAX_SPRING_COMP
        E_spring_max = 0.5 * K_SPRING * delta_r_max ** 2

        if E_spring_max >= E_jump:
            E_impulse = 0.0
            delta_r_des = math.sqrt(2 * E_jump / K_SPRING)
        else:
            E_impulse = E_jump - E_spring_max
            delta_r_des = delta_r_max

        if E_impulse > 0:
            delta_v_imp = math.sqrt(2 * E_impulse / M_BODY)
            delta_p = M_BODY * delta_v_imp
            omega_des = delta_p * R_PLECHO / (ETA * I_GR)
        else:
            omega_des = 0.0

        if omega_des < 1.0:
            omega_des = self.omega_res

        return omega_des, delta_r_des

    def update_phases(self, s):
        contact = s['contact']
    
        if self.phase == START:
            if contact:
                self.target_spin_speed, self.target_delta_r = self.calculate_setpoints(TARGET_HEIGHT)
                print(f"Target spin: {self.target_spin_speed:.1f} rad/s, Δr: {self.target_delta_r*1000:.1f} mm")
                self._set_phase(SPINUP)

        elif not contact and self.phase != FLIGHT:
            if self.vy > 0:
                self._set_phase(FLIGHT)
       
        elif self.phase == FLIGHT:
            #h = s['z'] - self.ground_z if self.ground_z else 0
            print(f'vy = {self.vy}')
            if self.vy < 0:
                self.target_hip_angle = self.compute_touchdown_angle()
                self._set_phase(LANDING)

        elif self.phase == SPINUP:
            '''
            h_des и v_des напрямую зависимы от vx vy, которые задаются
            в compute_stance_velocity
            '''
            if self.start_angle is None:
                self.start_angle = s['cent_angle']
            delta_r = s['compression']
            angle = s['hip_angle']
            if ((delta_r <= self.target_delta_r + self.target_delta_r * 0.1 and 
                delta_r >= self.target_delta_r - self.target_delta_r * 0.1) and
                (angle <= self.target_hip_angle + self.target_hip_angle * 0.1 and
                 angle >= self.target_hip_angle - self.target_hip_angle * 0.1)):
                self._set_phase(FLIGHT)
                
        
        elif self.phase == LANDING:
            if contact:
                self._set_phase(SPINUP)

    # ========== Управление центробежным грузом ==========
    def control_centrifugal(self):
        t = self.robot.getTime()
        if self.phase == SPINUP:
            err = self.target_spin_speed - self.omega
            tau = KP_SPIN * err
            tau = max(-GR_TORQUE_MAX, min(GR_TORQUE_MAX, tau))
            #self.centrifugal_motor.setTorque(tau)            -----------!!!
        elif self.phase == BRAKE_JUMP:
            if self.brake_start_time is None:
                self.brake_start_time = t
            if t - self.brake_start_time < BRAKE_DURATION:
                self.centrifugal_motor.setTorque(BRAKE_TORQUE)
            else:
                self.centrifugal_motor.setTorque(0.0)
        elif self.phase == LANDING:
            if abs(self.omega) > 0.5:
                tau = -KP_SPIN * self.omega
                tau = max(-GR_TORQUE_MAX, min(GR_TORQUE_MAX, tau))
                self.centrifugal_motor.setTorque(tau)
            else:
                self.centrifugal_motor.setTorque(0.0)
        else:
            self.centrifugal_motor.setTorque(0.0)

    # ========== Управление бедром ==========
    def control_hip(self):
        if self.phase == START:
            self.hip_motor.setTorque(0.0)
            return
        current = self.hip_sensor.getValue() if self.hip_sensor else 0.0
        if self.phase == SPINUP:
            self.target_hip_angle = BASE_HIP_ANGLE
        elif self.phase == LANDING:
            self.target_hip_angle = self.compute_touchdown_angle()
        dt = TIME_STEP / 1000.0
        err = self.target_hip_angle - current
        der = (err - self.prev_hip_error) / dt
        tau = KP_HIP * err + KD_HIP * der
        print(f'target angle = {self.target_hip_angle}, current angle = {current}')
        #print(f'tau = {KP_HIP} * {err} + {KD_HIP} * {der} = {tau}')
        tau = max(-HIP_TORQUE_MAX, min(HIP_TORQUE_MAX, tau))
        self.hip_motor.setTorque(tau)
        self.prev_hip_error = err

    # ========== Запись данных в CSV ==========
    def write_csv_data(self, s):
        """Записывает текущее состояние системы в CSV-файл."""
        t = self.robot.getTime()
        self.csv_writer.writerow([
            f"{t:.3f}",
            PHASE_NAMES[self.phase],
            f"{s['z']:.6f}",
            f"{s['x']:.6f}",
            f"{s['compression']:.6f}",
            f"{s['leg_len']:.6f}",
            f"{s['hip_angle']:.6f}",
            f"{self.target_hip_angle:.6f}",
            f"{self.vx:.6f}",
            f"{self.vy:.6f}",
            f"{self.omega:.6f}",
            f"{1 if s['contact'] else 0}"
        ])

    # ========== Служебные ==========
    def _set_phase(self, new_phase):
        t = self.robot.getTime()
        print(f"t={t:.2f}s | {PHASE_NAMES[self.phase]} -> {PHASE_NAMES[new_phase]}")
        self.phase = new_phase
        self.phase_start_time = t
        if new_phase == SPINUP:
            self.start_angle = None
            self.revolution_complete = False
        if new_phase == BRAKE_JUMP:
            self.brake_start_time = None

    def log_state(self, s):
        t = self.robot.getTime()
        if int(t * 1000) % 10 == 0:
            h = s['z'] #- self.ground_z if self.ground_z else 0
            if self.phase in (SPINUP, BRAKE_JUMP, LANDING):
                print(f"t={t:.2f}s | {PHASE_NAMES[self.phase]} | "
                      f"h={h:.3f}m | Δr={s['compression']:.4f}m | r_dot={self.leg_velocity:.2f} | "
                      f"phi={s['hip_angle']:.2f} | "
                      f"vx_s={self.vx_stance:.2f} | vy_s={self.vy_stance:.2f} | "
                      f"ω={self.omega:.1f}")
            else:
                print(f"t={t:.2f}s | {PHASE_NAMES[self.phase]} | "
                      f"h={h:.3f}m | ω={self.omega:.1f} | φ_ref={self.target_hip_angle:.3f} | "
                      f"φ={s['hip_angle']:.2f} | vx={self.vx:.2f}")

    def run(self):
        while self.robot.step(TIME_STEP) != -1:
            s = self.read_sensors()
            self.compute_omega(s)
            self.compute_velocity(s)
            if self.phase == SPINUP:
                self.compute_stance_kinematics(s)
                #self.compute_stance_velocity(s)
            if not s['contact']:
                self.compute_vy_no_contact(s)
            self.update_phases(s)
            self.control_centrifugal()
            self.control_hip()
            self.write_csv_data(s)  # Запись данных на каждом шаге
            self.log_state(s)
        
        # Закрытие файла при завершении симуляции
        self.csv_file.close()
        print("CSV file saved: sahr_data.csv")

if __name__ == "__main__":
    SAHRController().run()