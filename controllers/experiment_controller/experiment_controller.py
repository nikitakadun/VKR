"""Эксперимент №1: Пассивное падение робота с фиксированными углами ноги.
Горизонтальное перемещение измеряется через угол рычага.
GPS используется только для высоты.
Добавлена компенсация начального угла рычага (18.7°).
"""

from controller import Robot, Motor, PositionSensor, TouchSensor, GPS, InertialUnit
import math

# ===== НАСТРОЙКИ =====
NOMINAL_LEG_LENGTH = 0.1      # метры (номинальная длина ноги)
ARM_LENGTH = 0.45             # метры (длина рычага от центра вращения до корпуса)
SIGN_FACTOR = 1               # знак для смещения стопы (+1 или -1)
ARM_ANGLE_OFFSET_DEG = 18.7   # начальный угол рычага для компенсации (градусы)
# =====================

def run_experiment(desired_angle_deg, duration=4.0, filename=None):
    if filename is None:
        filename = f"experiment_fall_angle_{desired_angle_deg:+.0f}.csv"
    
    robot = Robot()
    time_step = int(robot.getBasicTimeStep())
    print(f"Базовый шаг времени: {time_step} мс")
    
    # --- Получение устройств ---
    hip_motor = robot.getDevice("hip_motor")
    centrifugal_motor = robot.getDevice("centrifugal_motor")
    leg_sensor = robot.getDevice("leg_sensor")
    hip_sensor = robot.getDevice("hip_sensor")
    foot_sensor = robot.getDevice("foot_sensor")
    body_gps = robot.getDevice("body_gps")
    inertial_unit = robot.getDevice("inertial_unit")
    arm_sensor = robot.getDevice("arm_angle_sensor")
    
    # Проверка наличия датчика рычага
    if arm_sensor is None:
        print("❌ arm_angle_sensor не найден! Добавьте PositionSensor на HingeJoint рычага.")
        return
    else:
        print("✅ arm_angle_sensor найден")
    
    # Включение датчиков
    if leg_sensor:
        leg_sensor.enable(time_step)
    if hip_sensor:
        hip_sensor.enable(time_step)
    if foot_sensor:
        foot_sensor.enable(time_step)
    if body_gps:
        body_gps.enable(time_step)
    if inertial_unit:
        inertial_unit.enable(time_step)
    if arm_sensor:
        arm_sensor.enable(time_step)
    
    # --- Отключение моторов ---
    if hip_motor:
        hip_motor.setPosition(float('inf'))
        hip_motor.setTorque(0.0)
    if centrifugal_motor:
        centrifugal_motor.setPosition(float('inf'))
        centrifugal_motor.setTorque(0.0)
    
    # --- Установка начального угла ноги ---
    if hip_motor:
        desired_angle_rad = math.radians(desired_angle_deg)
        hip_motor.setPosition(desired_angle_rad)
        for _ in range(20):
            robot.step(time_step)
        hip_motor.setTorque(0.0)
        hip_motor.setPosition(float('inf'))
    
    # --- Калибровка: начальный угол рычага для компенсации ---
    arm_angle_offset_rad = math.radians(ARM_ANGLE_OFFSET_DEG)
    print(f"Начальный угол рычага (компенсация): {ARM_ANGLE_OFFSET_DEG:.1f}° ({arm_angle_offset_rad:.4f} рад)")
    
    # --- Подготовка к записи данных ---
    max_steps = int(duration * 1000 / time_step)
    
    with open(filename, 'w') as f:
        f.write("time,leg_length,hip_angle_rad,hip_angle_deg,contact,"
                "height_from_gps,"
                "arm_angle_raw_rad,arm_angle_rad,arm_angle_deg,x_displacement,"
                "foot_x_computed,foot_z_computed,"
                "roll,pitch,yaw\n")
        
        print(f"\nЗапуск эксперимента с углом ноги {desired_angle_deg}°")
        print(f"Номинальная длина ноги: {NOMINAL_LEG_LENGTH} м")
        print(f"Длина рычага (ARM_LENGTH): {ARM_LENGTH} м")
        
        print("\n=== РЕЗУЛЬТАТЫ ИЗМЕРЕНИЙ ===")
        print("Формат: [время] | контакт | высота | X_перемещ | угол_рычага(скорр) | угол_ноги | leg_len")
        print("-" * 110)
        
        t = 0.0
        last_print = 0.0
        
        for step in range(max_steps):
            # --- Чтение датчиков ---
            
            # leg_sensor
            if leg_sensor:
                leg_offset = leg_sensor.getValue()
                leg_len = NOMINAL_LEG_LENGTH + leg_offset
            else:
                leg_len = NOMINAL_LEG_LENGTH
            
            # hip_sensor (угол ноги)
            if hip_sensor:
                hip_angle_rad = hip_sensor.getValue()
                hip_angle_deg = math.degrees(hip_angle_rad)
            else:
                hip_angle_rad = math.radians(desired_angle_deg)
                hip_angle_deg = desired_angle_deg
            
            # foot_sensor
            if foot_sensor:
                is_contact = 1 if foot_sensor.getValue() > 0.5 else 0
            else:
                is_contact = 0
            
            # GPS (только для высоты)
            if body_gps:
                gps = body_gps.getValues()
                height = gps[2]  # Z координата
            else:
                height = 0
            
            # arm_sensor (угол рычага) — с компенсацией начального смещения
            if arm_sensor:
                arm_angle_raw_rad = arm_sensor.getValue()
                # КОМПЕНСАЦИЯ: вычитаем начальный угол
                arm_angle_rad = arm_angle_raw_rad - arm_angle_offset_rad
                arm_angle_deg = math.degrees(arm_angle_rad)
                # Горизонтальное перемещение (проекция на ось X)
                x_displacement = ARM_LENGTH * math.sin(arm_angle_rad)
            else:
                arm_angle_raw_rad = 0.0
                arm_angle_rad = 0.0
                arm_angle_deg = 0.0
                x_displacement = 0.0
            
            # --- ВЫЧИСЛЕНИЕ ПОЛОЖЕНИЯ СТОПЫ (для информации) ---
            foot_offset_x = SIGN_FACTOR * leg_len * math.sin(hip_angle_rad)
            foot_offset_z = -leg_len * math.cos(hip_angle_rad)
            
            # Абсолютное положение стопы (через перемещение от рычага)
            foot_x_abs = x_displacement + foot_offset_x
            foot_z_abs = height + foot_offset_z
            
            # IMU
            if inertial_unit:
                roll_pitch_yaw = inertial_unit.getRollPitchYaw()
            else:
                roll_pitch_yaw = [0, 0, 0]
            
            # Запись в файл
            f.write(f"{t:.6f},{leg_len:.6f},{hip_angle_rad:.6f},{hip_angle_deg:.3f},"
                   f"{is_contact},{height:.6f},"
                   f"{arm_angle_raw_rad:.6f},{arm_angle_rad:.6f},{arm_angle_deg:.3f},{x_displacement:.6f},"
                   f"{foot_x_abs:.6f},{foot_z_abs:.6f},"
                   f"{roll_pitch_yaw[0]:.6f},{roll_pitch_yaw[1]:.6f},{roll_pitch_yaw[2]:.6f}\n")
            
            # --- ВЫВОД В КОНСОЛЬ (каждые 0.1 с) ---
            if t - last_print >= 0.1 or step == 0:
                last_print = t
                contact_symbol = "🟢" if is_contact else "⚪"
                
                print(f"[{t:.2f} с] {contact_symbol} | "
                      f"высота: {height:5.3f} м | "
                      f"X_перем: {x_displacement:6.3f} м | "
                      f"угол_рыч: {arm_angle_deg:5.1f}° | "
                      f"угол_ноги: {hip_angle_deg:5.1f}° | "
                      f"leg: {leg_len:.3f} м")
            
            robot.step(time_step)
            t += time_step / 1000.0
    
    print(f"\n✅ Эксперимент завершен! Данные сохранены в {filename}")
    
    # --- Итоговая статистика ---
    print("\n=== ИТОГОВАЯ СТАТИСТИКА ===")
    print(f"Компенсация начального угла рычага: {ARM_ANGLE_OFFSET_DEG:.1f}°")
    print(f"Конечный угол рычага (скорректированный): {arm_angle_deg:.1f}°")
    print(f"Максимальное перемещение X: {x_displacement:.3f} м")
    print(f"Длина рычага: {ARM_LENGTH} м")
    
    return filename


if __name__ == "__main__":

    # Эксперимент с углом ноги 0 градусов
    run_experiment(desired_angle_deg=0, duration=2.5)