# controllers/sahr_jump_controller/sahr_jump_controller.py

from controller import Robot
import math

class CentrifugalJumpController:
    def __init__(self):
        self.robot = Robot()
        self.timestep = int(self.robot.getBasicTimeStep())
        
        # ===== УСТРОЙСТВА =====
        self.centrifugal_motor = self.robot.getDevice('centrifugal_motor')
        if self.centrifugal_motor:
            self.centrifugal_motor.setPosition(float('inf'))
            self.centrifugal_motor.setVelocity(0.0)
        
        self.centrifugal_sensor = self.robot.getDevice('centrifugal_sensor')
        if self.centrifugal_sensor:
            self.centrifugal_sensor.enable(self.timestep)
        
        self.hip_motor = self.robot.getDevice('hip_motor')
        if self.hip_motor:
            self.hip_motor.setPosition(float('inf'))
            self.hip_motor.setVelocity(0.0)
        
        self.hip_sensor = self.robot.getDevice('hip_sensor')
        if self.hip_sensor:
            self.hip_sensor.enable(self.timestep)
        
        self.leg_sensor = self.robot.getDevice('leg_sensor')
        if self.leg_sensor:
            self.leg_sensor.enable(self.timestep)
        
        self.foot_sensor = self.robot.getDevice('foot_sensor')
        if self.foot_sensor:
            self.foot_sensor.enable(self.timestep)
        
        self.gps = self.robot.getDevice('body_gps')
        if self.gps:
            self.gps.enable(self.timestep)
        
        self.height_sensor = self.robot.getDevice('height_sensor')
        if self.height_sensor:
            self.height_sensor.enable(self.timestep)
        
        self.imu = self.robot.getDevice('inertial_unit')
        if self.imu:
            self.imu.enable(self.timestep)
        
        # ===== ПАРАМЕТРЫ =====
        self.max_torque = 22.0
        self.leg_length = 0.275
        
        # Параметры центробежного прыжка
        self.spin_velocity = 45.0
        self.spin_time = 2.5
        
        # ===== ФИНАЛЬНЫЕ ОПТИМИЗИРОВАННЫЕ УГЛЫ =====
        self.back_angle = -0.3      # -17° (минимальный замах)
        self.push_angle = 1.3       # 74° (агрессивный толчок)
        self.stabilize_angle = 0.4  # 23°
        
        # Порог отрыва (увеличен)
        self.MIN_TAKEOFF_ANGLE = 0.25  # 14.3° - ждем большего угла
        
        # Параметры высоты
        self.GROUND_HEIGHT = 0.03
        self.AIR_HEIGHT = 0.05
        
        # ===== ОПТИМИЗАЦИЯ =====
        self.TARGET_TOUCHDOWN_ANGLE = 0.0
        self.TOUCHDOWN_PREPARE_HEIGHT = 0.10
        self.angle_gain_flight = 8.0
        self.angle_gain_touchdown = 12.0
        
        # ===== ПАРАМЕТРЫ =====
        self.gravity = 9.81
        self.robot_mass = 10.0
        self.spring_constant = 50000.0
        self.STABILIZATION_TIME = 0.08  # Уменьшено для скорости
        self.PREPARE_TIME = 0.08
        
        # ===== ПЕРЕМЕННЫЕ =====
        self.last_x = 0
        self.last_y = 0
        self.last_time = 0
        self.horizontal_speed = 0
        self.vertical_speed = 0
        
        self.takeoff_time = 0
        self.takeoff_vertical_speed = 0
        self.takeoff_horizontal_speed = 0
        
        self.was_in_air = False
        self.was_on_ground = True
        self.apex_height = 0
        self.apex_printed = False
        
        self.sensor_calibrated = False
        self.ground_height_at_start = 0
        
        self.phase = "PREPARE"
        self.phase_start = 0
        self.jump_count = 0
        self.ground_stable_start = 0
        
        self.last_takeoff_time = 0
        self.last_touchdown_time = 0
        self.takeoff_cooldown = 0.25
        self.touchdown_cooldown = 0.08
        
        self.filtered_height = 0
        self.filter_alpha = 0.5
        
        self.max_compression = 0
        self.last_print_time = 0
        
        print("\n" + "="*80)
        print("SAHR CENTRIFUGAL JUMP CONTROLLER - FINAL OPTIMIZED")
        print("="*80)
        print(f"Back angle: {self.back_angle*180/math.pi:.0f}°")
        print(f"Push angle: {self.push_angle*180/math.pi:.0f}°")
        print(f"Min takeoff angle: {self.MIN_TAKEOFF_ANGLE*180/math.pi:.0f}°")
        print(f"Max torque: {self.max_torque} Nm")
        print("="*80)
    
    def calibrate_sensor(self):
        if self.height_sensor and not self.sensor_calibrated:
            readings = []
            for _ in range(10):
                raw = self.height_sensor.getValue()
                if raw > 0 and raw < 0.5:
                    readings.append(raw)
                self.robot.step(self.timestep)
            
            if readings:
                self.ground_height_at_start = sum(readings) / len(readings)
                self.sensor_calibrated = True
                print(f"Height sensor calibrated: ground at {self.ground_height_at_start:.3f}m")
    
    def get_state(self):
        phi = self.hip_sensor.getValue() if self.hip_sensor else 0
        r = self.leg_sensor.getValue() if self.leg_sensor else self.leg_length
        
        compression = max(0, self.leg_length - r)
        
        sensor_height = 0.1
        if self.height_sensor:
            raw = self.height_sensor.getValue()
            if raw > 0 and raw < 0.5:
                sensor_height = raw
            else:
                sensor_height = self.filtered_height if self.filtered_height > 0 else 0.05
        
        if self.filtered_height == 0:
            self.filtered_height = sensor_height
        else:
            self.filtered_height = self.filter_alpha * sensor_height + (1 - self.filter_alpha) * self.filtered_height
        
        if self.sensor_calibrated:
            height = max(0, self.filtered_height - self.ground_height_at_start)
        else:
            height = 0
        
        gps_x = 0
        if self.gps:
            pos = self.gps.getValues()
            gps_x = pos[0]
        
        return {
            'phi': phi,
            'compression': compression,
            'height': height,
            'x': gps_x,
            'time': self.robot.getTime()
        }
    
    def calculate_speed(self, current_x, current_y, current_time):
        dt = current_time - self.last_time
        if dt > 0.001:
            self.horizontal_speed = (current_x - self.last_x) / dt
            self.vertical_speed = (current_y - self.last_y) / dt
        
        self.last_x = current_x
        self.last_y = current_y
        self.last_time = current_time
    
    def run(self):
        print("Starting initialization...")
        
        start = self.robot.getTime()
        while self.robot.getTime() - start < 1.0:
            self.robot.step(self.timestep)
        
        self.calibrate_sensor()
        
        while self.robot.getTime() - start < 1.5:
            self.robot.step(self.timestep)
        
        print("Control loop started\n")
        print("*** FINAL OPTIMIZED - Maximum forward energy transfer ***\n")
        
        self.phase = "PREPARE"
        self.phase_start = self.robot.getTime()
        self.ground_stable_start = self.robot.getTime()
        self.last_print_time = self.robot.getTime()
        
        while self.robot.step(self.timestep) != -1:
            state = self.get_state()
            t = state['time']
            dt = t - self.phase_start
            
            self.calculate_speed(state['x'], state['height'], t)
            
            if state['compression'] > self.max_compression:
                self.max_compression = state['compression']
            
            is_on_ground = (state['height'] < self.GROUND_HEIGHT)
            is_in_air = (state['height'] > self.AIR_HEIGHT)
            
            if not is_on_ground and not is_in_air:
                is_on_ground = self.was_on_ground
                is_in_air = self.was_in_air
            
            if is_in_air and state['height'] > self.apex_height:
                self.apex_height = state['height']
                self.apex_printed = False
            
            # ===== ВЗЛЕТ =====
            if is_in_air and not self.was_in_air:
                if t - self.last_takeoff_time > self.takeoff_cooldown:
                    self.last_takeoff_time = t
                    self.takeoff_time = t
                    self.takeoff_vertical_speed = self.vertical_speed
                    self.takeoff_horizontal_speed = self.horizontal_speed
                    
                    self.jump_count += 1
                    print(f"\n[{t:.2f}] TAKEOFF #{self.jump_count} | angle={state['phi']*180/math.pi:.1f}° | vx={self.horizontal_speed:+.2f}")
                    
                    self.phase = "FLIGHT"
                    self.phase_start = t
                    if self.centrifugal_motor:
                        self.centrifugal_motor.setVelocity(0)
                    self.max_compression = 0
                    self.apex_height = state['height']
                    self.apex_printed = False
            
            # ===== ПОСАДКА =====
            if is_on_ground and self.was_in_air:
                if t - self.last_touchdown_time > self.touchdown_cooldown:
                    self.last_touchdown_time = t
                    
                    print(f"\n[{t:.2f}] TOUCHDOWN #{self.jump_count} | angle={state['phi']*180/math.pi:.1f}° | jump={self.apex_height:.3f}m | vx={self.horizontal_speed:+.2f}")
                    
                    self.phase = "LAND"
                    self.phase_start = t
                    self.ground_stable_start = t
                    self.apex_height = 0
                    self.max_compression = 0
                    self.apex_printed = False
            
            self.was_on_ground = is_on_ground
            self.was_in_air = is_in_air
            
            # ===== АВТОМАТ СОСТОЯНИЙ =====
            
            if self.phase == "PREPARE":
                torque = 3.0 * (0.0 - state['phi'])
                
                if is_on_ground and (t - self.ground_stable_start) > self.PREPARE_TIME:
                    self.phase = "SPIN_UP"
                    self.phase_start = t
                    self.jump_count += 1
                    print(f"\n[{t:.2f}] SPIN_UP (jump #{self.jump_count})")
            
            elif self.phase == "SPIN_UP":
                if self.centrifugal_motor:
                    self.centrifugal_motor.setVelocity(self.spin_velocity)
                
                torque = 3.0 * (0.0 - state['phi'])
                
                if is_on_ground and dt >= self.spin_time:
                    self.phase = "JUMP"
                    self.phase_start = t
                    print(f"\n[{t:.2f}] JUMP #{self.jump_count} | push={self.push_angle*180/math.pi:.0f}°")
                elif not is_on_ground:
                    print(f"\n[{t:.2f}] Early takeoff!")
                    self.phase = "FLIGHT"
                    self.phase_start = t
                    if self.centrifugal_motor:
                        self.centrifugal_motor.setVelocity(0)
            
            elif self.phase == "JUMP":
                if is_on_ground:
                    # ===== ФАЗЫ ПРЫЖКА С УДЕРЖАНИЕМ =====
                    if dt < 0.15:
                        # Минимальный замах
                        target = self.back_angle
                        error = target - state['phi']
                        torque = 15.0 * error
                        
                    elif dt < 1.0:
                        # Длинный мощный толчок
                        target = self.push_angle
                        error = target - state['phi']
                        torque = 35.0 * error
                        
                        # КЛЮЧЕВОЕ: удерживаем на земле до достижения угла
                        if state['phi'] < self.MIN_TAKEOFF_ANGLE:
                            # Принудительное удержание ноги на земле
                            # Увеличиваем момент, чтобы предотвратить отрыв
                            torque = max(torque, 30.0)
                            
                    elif dt < 1.3:
                        # Финальный толчок и отрыв
                        target = self.stabilize_angle
                        error = target - state['phi']
                        torque = 15.0 * error
                        
                    else:
                        self.phase = "FLIGHT"
                        self.phase_start = t
                        if self.centrifugal_motor:
                            self.centrifugal_motor.setVelocity(0)
                        print(f"\n[{t:.2f}] FLIGHT (timeout)")
                else:
                    torque = 4.0 * (0.0 - state['phi'])
            
            elif self.phase == "FLIGHT":
                flight_time = t - self.phase_start
                
                if not self.apex_printed and flight_time > 0.2 and self.vertical_speed < 0 and self.apex_height > 0.01:
                    print(f"  apex: {self.apex_height:.3f}m")
                    self.apex_printed = True
                
                # Управление углом в полете
                if state['height'] < self.TOUCHDOWN_PREPARE_HEIGHT:
                    target_angle = 0.0
                    gain = self.angle_gain_touchdown
                else:
                    target_angle = 0.0
                    gain = self.angle_gain_flight
                
                error = target_angle - state['phi']
                torque = gain * error
                torque = max(-self.max_torque, min(self.max_torque, torque))
                
                if state['height'] < self.TOUCHDOWN_PREPARE_HEIGHT / 2:
                    torque = torque * 0.6
            
            elif self.phase == "LAND":
                torque = 4.0 * (0.0 - state['phi'])
                
                if dt > self.STABILIZATION_TIME:
                    self.phase = "PREPARE"
                    self.phase_start = t
                    self.ground_stable_start = t
                    print(f"\n[{t:.2f}] PREPARE (ready)")
            
            else:
                torque = 0
            
            if self.hip_motor:
                torque = max(-self.max_torque, min(self.max_torque, torque))
                self.hip_motor.setTorque(torque)
            
            # ===== ВЫВОД =====
            if t - self.last_print_time >= 0.2:
                phase_map = {
                    "PREPARE": "PREP",
                    "SPIN_UP": "SPIN",
                    "JUMP": "JMP",
                    "FLIGHT": "FLY",
                    "LAND": "LAND"
                }
                phase_name = phase_map.get(self.phase, self.phase[:4])
                ground_char = 'G' if is_on_ground else 'A'
                
                print(f"[{t:.1f}s] {phase_name:4s} {ground_char} | "
                      f"angle={state['phi']*180/math.pi:+5.1f}° | "
                      f"h={state['height']:.3f}m | "
                      f"comp={state['compression']*1000:3.0f}mm | "
                      f"vx={self.horizontal_speed:+.2f}")
                
                self.last_print_time = t


if __name__ == "__main__":
    controller = CentrifugalJumpController()
    controller.run()