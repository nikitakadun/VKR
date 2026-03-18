# controllers/sahr_jump_controller/sahr_jump_controller.py

from controller import Robot
import math

class CentrifugalJumpController:
    def __init__(self):
        self.robot = Robot()
        self.timestep = int(self.robot.getBasicTimeStep())
        
        # ===== УСТРОЙСТВА =====
        # Центробежный механизм
        self.centrifugal_motor = self.robot.getDevice('centrifugal_motor')
        if self.centrifugal_motor:
            self.centrifugal_motor.setPosition(float('inf'))
            self.centrifugal_motor.setVelocity(0.0)
        
        self.centrifugal_sensor = self.robot.getDevice('centrifugal_sensor')
        if self.centrifugal_sensor:
            self.centrifugal_sensor.enable(self.timestep)
        
        # Основные моторы
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
        
        # ===== ПАРАМЕТРЫ =====
        self.max_torque = 5.5
        self.leg_length = 0.275
        self.gravity = 9.81
        
        # Параметры центробежного прыжка
        self.spin_velocity = 30.0      # скорость раскрутки (рад/с)
        self.spin_time = 2.0            # время раскрутки (сек)
        self.push_angle = 0.6           # угол толчка (+34°)
        self.back_angle = -0.4           # угол замаха (-23°)
        
        # Состояния
        self.phase = "SPIN_UP"
        self.phase_start = 0
        self.jump_count = 0
        
        print("Центробежный прыгающий контроллер запущен")
        print("SPIN_UP -> JUMP -> RECOVER")
    
    def get_state(self):
        """Получение состояния"""
        phi = self.hip_sensor.getValue() if self.hip_sensor else 0
        r = self.leg_sensor.getValue() if self.leg_sensor else self.leg_length
        foot = self.foot_sensor.getValue() if self.foot_sensor else 0
        centrifugal_angle = self.centrifugal_sensor.getValue() if self.centrifugal_sensor else 0
        
        compression = max(0, self.leg_length - r)
        on_ground = (compression > 0.001) or (foot > 0.1)
        
        return {
            'phi': phi,
            'compression': compression,
            'foot': foot,
            'on_ground': on_ground,
            'centrifugal_angle': centrifugal_angle,
            'time': self.robot.getTime()
        }
    
    def run(self):
        print("Старт через 2 секунды...")
        
        # Ждем 2 секунды
        start = self.robot.getTime()
        while self.robot.getTime() - start < 2.0:
            self.robot.step(self.timestep)
        
        print("НАЧИНАЕМ!")
        self.phase = "SPIN_UP"
        self.phase_start = self.robot.getTime()
        
        while self.robot.step(self.timestep) != -1:
            state = self.get_state()
            t = state['time']
            dt = t - self.phase_start
            
            # ===== АВТОМАТ СОСТОЯНИЙ =====
            
            if self.phase == "SPIN_UP":
                # Фаза 1: Раскрутка маховика (2 секунды)
                if dt < self.spin_time:
                    # Раскручиваем центробежный механизм
                    self.centrifugal_motor.setVelocity(self.spin_velocity)
                    
                    # Держим ногу вертикально
                    torque = 2.0 * (0.0 - state['phi'])
                    
                else:
                    self.phase = "JUMP"
                    self.phase_start = t
                    self.jump_count += 1
                    print(f"Прыжок #{self.jump_count}")
            
            elif self.phase == "JUMP":
                # Фаза 2: Резкий прыжок
                if dt < 0.4:
                    # Замах назад
                    target = self.back_angle
                    error = target - state['phi']
                    torque = 5.0 * error
                    
                elif dt < 0.8:
                    # Мощный толчок вперед
                    target = self.push_angle
                    error = target - state['phi']
                    torque = 8.0 * error
                    
                elif dt < 1.2:
                    # Завершение прыжка
                    target = 0.0
                    error = target - state['phi']
                    torque = 3.0 * error
                    
                else:
                    self.phase = "RECOVER"
                    self.phase_start = t
                    print("Приземление")
                    
                    # Останавливаем центробежный мотор
                    self.centrifugal_motor.setVelocity(0)
            
            elif self.phase == "RECOVER":
                # Фаза 3: Восстановление (2 секунды)
                if dt < 2.0:
                    # Возвращаем ногу в вертикальное положение
                    torque = 2.0 * (0.0 - state['phi'])
                    
                else:
                    self.phase = "SPIN_UP"
                    self.phase_start = t
                    print("Новый цикл - раскрутка")
            
            else:
                torque = 0
            
            # Применяем момент к ноге
            if self.hip_motor:
                torque = max(-self.max_torque, min(self.max_torque, torque))
                self.hip_motor.setTorque(torque)

# Запуск
if __name__ == "__main__":
    controller = CentrifugalJumpController()
    controller.run()