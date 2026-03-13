from controller import Robot
import math

class SAHRController:
    def __init__(self):
        self.robot = Robot()
        self.timestep = int(self.robot.getBasicTimeStep())
        
        # ===== ПАРАМЕТРЫ ИЗ СТАТЬИ =====
        self.leg_length = 0.275        # длина ноги в покое
        self.leg_stiffness = 4800       # жесткость пружины
        self.max_torque = 5.5           # максимальный момент hip_motor
        self.leg_mass = 0.5             # масса ноги
        self.body_mass = 5.0            # масса тела (из вашего world)
        
        # ===== ЦЕЛЕВЫЕ ПАРАМЕТРЫ =====
        self.desired_height = 0.29       # желаемая высота прыжка
        self.desired_speed = 0.7         # желаемая скорость
        
        # ===== ПОЛУЧАЕМ УСТРОЙСТВА =====
        # Hip motor (единственный мотор для прыжков)
        self.hip_motor = self.robot.getDevice('hip_motor')
        if self.hip_motor:
            self.hip_motor.setPosition(float('inf'))
            self.hip_motor.setVelocity(0.0)
            print("✓ hip_motor найден")
        
        # Моторы BallJoint (для управления штангой)
        self.ball1_x = self.robot.getDevice('ball1_motor_x')
        self.ball1_y = self.robot.getDevice('ball1_motor_y')
        self.ball1_z = self.robot.getDevice('ball1_motor_z')
        
        self.ball2_x = self.robot.getDevice('ball2_motor_x')
        self.ball2_y = self.robot.getDevice('ball2_motor_y')
        self.ball2_z = self.robot.getDevice('ball2_motor_z')
        
        # Настраиваем моторы BallJoint (делаем их свободными)
        for motor in [self.ball1_x, self.ball1_y, self.ball1_z,
                     self.ball2_x, self.ball2_y, self.ball2_z]:
            if motor:
                motor.setPosition(float('inf'))
                motor.setVelocity(0.0)
                print(f"✓ BallJoint motor найден")
        
        # Сенсоры
        self.hip_sensor = self.robot.getDevice('hip_sensor')
        if self.hip_sensor:
            self.hip_sensor.enable(self.timestep)
            print("✓ hip_sensor найден")
        
        self.leg_sensor = self.robot.getDevice('leg_sensor')
        if self.leg_sensor:
            self.leg_sensor.enable(self.timestep)
            print("✓ leg_sensor найден")
        
        self.foot_sensor = self.robot.getDevice('foot_sensor')
        if self.foot_sensor:
            self.foot_sensor.enable(self.timestep)
            print("✓ foot_sensor найден")
        
        self.body_imu = self.robot.getDevice('body_imu')
        if self.body_imu:
            self.body_imu.enable(self.timestep)
            print("✓ body_imu найден")
        
        # ===== СОСТОЯНИЕ РОБОТА =====
        self.phase = "INIT"
        self.jump_count = 0
        self.phase_start = 0
        self.compression_max = 0
        self.last_foot_force = 0
        
        # Для вычисления производных
        self.prev_phi = 0
        self.prev_r = self.leg_length
        self.prev_time = self.robot.getTime()
        
        print("\n" + "="*60)
        print("🦘 SAHR CONTROLLER FOR TWO BALLJOINT ROBOT")
        print("="*60)
        print(f"Desired height: {self.desired_height}m")
        print(f"Desired speed: {self.desired_speed}m/s")
        print(f"Leg stiffness: {self.leg_stiffness}N/m")
        print("="*60)
    
    def get_state(self):
        """Получение полного состояния робота"""
        current_time = self.robot.getTime()
        dt = current_time - self.prev_time
        
        # Данные с сенсоров
        phi = self.hip_sensor.getValue() if self.hip_sensor else 0
        r = self.leg_sensor.getValue() if self.leg_sensor else self.leg_length
        
        # Вычисляем производные
        if dt > 0.001:
            phi_dot = (phi - self.prev_phi) / dt
            r_dot = (r - self.prev_r) / dt
        else:
            phi_dot = 0
            r_dot = 0
        
        # Сжатие ноги
        compression = max(0, self.leg_length - r)
        
        # Сила в стопе
        foot_force = self.foot_sensor.getValue() if self.foot_sensor else 0
        self.last_foot_force = foot_force
        
        # Касание земли (по сжатию или по силе)
        on_ground = (compression > 0.002) or (foot_force > 0.5)
        
        # Сохраняем для следующего шага
        self.prev_phi = phi
        self.prev_r = r
        self.prev_time = current_time
        
        # Положение и скорость тела (упрощенно)
        # y ≈ r * cos(phi) - для вертикальной координаты
        y = r * math.cos(phi)
        
        # Вертикальная скорость
        y_dot = r_dot * math.cos(phi) - r * phi_dot * math.sin(phi)
        
        # Горизонтальная скорость (приблизительно)
        x_dot = -r_dot * math.sin(phi) - r * phi_dot * math.cos(phi)
        
        return {
            'phi': phi,
            'phi_dot': phi_dot,
            'r': r,
            'r_dot': r_dot,
            'compression': compression,
            'foot_force': foot_force,
            'on_ground': on_ground,
            'y': y,
            'y_dot': y_dot,
            'x_dot': x_dot,
            'time': current_time
        }
    
    def control_balljoints(self, state, jump_phase):
        """Управление двумя BallJoint на штанге"""
        # Используем BallJoint для балансировки и создания инерции
        
        if not (self.ball1_y and self.ball2_y):
            return
        
        t = state['time']
        
        if jump_phase == "PRE_JUMP":
            # Перед прыжком - отводим штангу назад для замаха
            angle1 = -0.2 * math.sin(t * 5)
            angle2 = -0.15 * math.sin(t * 5 + 0.5)
            
        elif jump_phase == "JUMP":
            # Во время прыжка - штанга создает момент инерции
            angle1 = 0.3 * math.sin(t * 8)
            angle2 = 0.25 * math.sin(t * 8 + 1.0)
            
        elif jump_phase == "FLIGHT":
            # В полете - стабилизация
            angle1 = -0.1 * state['x_dot']
            angle2 = -0.08 * state['x_dot']
            
        else:  # STANCE
            # В опоре - помогаем отталкиванию
            angle1 = 0.15 * state['compression'] * 100
            angle2 = 0.12 * state['compression'] * 100
        
        # Ограничиваем углы
        angle1 = max(-0.5, min(0.5, angle1))
        angle2 = max(-0.5, min(0.5, angle2))
        
        # Применяем к моторам (скоростное управление)
        self.ball1_y.setVelocity(5.0 * (angle1 - state['phi']))
        self.ball2_y.setVelocity(4.0 * (angle2 - state['phi']))
        
        # Z-моторы для вращения
        if self.ball1_z and self.ball2_z:
            self.ball1_z.setVelocity(2.0 * math.sin(t * 3))
            self.ball2_z.setVelocity(1.5 * math.cos(t * 3))
    
    def run(self):
        """Главный цикл"""
        print("\n🚀 Запуск через 2 секунды...")
        
        # Ждем 2 секунды
        start = self.robot.getTime()
        while self.robot.getTime() - start < 2.0:
            self.robot.step(self.timestep)
        
        print("\n🦘 НАЧИНАЕМ ПРЫЖКИ!")
        self.phase = "INIT"
        self.phase_start = self.robot.getTime()
        
        last_print = 0
        jump_subphase = "PRE_JUMP"
        
        while self.robot.step(self.timestep) != -1:
            state = self.get_state()
            t = state['time']
            time_in_phase = t - self.phase_start
            
            # ===== АВТОМАТ СОСТОЯНИЙ =====
            
            if self.phase == "INIT":
                # Начальная раскачка для первого прыжка
                if time_in_phase < 0.3:
                    # Отводим ногу назад
                    torque = 2.0 * (-0.2 - state['phi'])
                    jump_subphase = "PRE_JUMP"
                    
                elif time_in_phase < 0.6:
                    # Резко бьем вперед
                    torque = 8.0 * (0.25 - state['phi'])
                    jump_subphase = "JUMP"
                    
                else:
                    torque = 0
                    if not state['on_ground']:
                        self.phase = "FLIGHT"
                        self.phase_start = t
                        jump_subphase = "FLIGHT"
                        print(f"\n🦘 ПЕРВЫЙ ПРЫЖОК!")
            
            elif self.phase == "FLIGHT":
                # В полете - готовим угол касания
                target_angle = -0.15 * self.desired_speed
                angle_error = target_angle - state['phi']
                torque = 1.5 * angle_error
                jump_subphase = "FLIGHT"
                
                # Переход в опору при касании
                if state['on_ground']:
                    self.phase = "STANCE"
                    self.phase_start = t
                    self.jump_count += 1
                    self.compression_max = 0
                    jump_subphase = "STANCE"
                    print(f"\n🦵 КАСАНИЕ #{self.jump_count}")
            
            elif self.phase == "STANCE":
                # В опоре - накапливаем энергию в пружине
                
                # Запоминаем максимальное сжатие
                self.compression_max = max(self.compression_max, state['compression'])
                
                if time_in_phase < 0.15:
                    # Первая фаза - даем ноге сжаться
                    torque = 0
                    jump_subphase = "COMPRESS"
                    
                elif time_in_phase < 0.35:
                    # Вторая фаза - добавляем момент для ускорения
                    # Чем больше сжатие, тем сильнее толчок
                    boost = self.compression_max * 30
                    torque = 2.5 + boost
                    jump_subphase = "BOOST"
                    
                    # Контроль скорости (уравнение из статьи)
                    phi_dot_des = self.desired_speed / self.leg_length
                    speed_error = state['phi_dot'] - phi_dot_des
                    torque += -0.5 * speed_error
                    
                else:
                    # Третья фаза - подготовка к отрыву
                    torque = 1.0
                    jump_subphase = "LIFTOFF"
                    
                    # Проверяем отрыв
                    if not state['on_ground']:
                        self.phase = "FLIGHT"
                        self.phase_start = t
                        print(f"   ВЗЛЕТ! Сжатие: {self.compression_max*1000:.1f}мм")
                        self.compression_max = 0
            
            else:
                torque = 0
            
            # ===== УПРАВЛЕНИЕ BALLJOINT =====
            self.control_balljoints(state, jump_subphase)
            
            # ===== ПРИМЕНЯЕМ МОМЕНТ К HIP MOTOR =====
            if self.hip_motor:
                torque = max(-self.max_torque, min(self.max_torque, torque))
                self.hip_motor.setTorque(torque)
            
            # ===== СТАТУС =====
            if t - last_print > 2.0:
                print(f"\n📊 t={t:.1f}s | Фаза: {self.phase}/{jump_subphase}")
                print(f"   Угол ноги: {state['phi']*180/math.pi:.1f}°")
                print(f"   Сжатие: {state['compression']*1000:.1f}мм")
                print(f"   Сила в стопе: {state['foot_force']:.1f}Н")
                print(f"   На земле: {state['on_ground']}")
                print(f"   Прыжков: {self.jump_count}")
                last_print = t

# Запуск
if __name__ == "__main__":
    controller = SAHRController()
    controller.run()