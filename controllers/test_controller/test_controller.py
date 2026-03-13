"""
ПРОСТОЙ КОНТРОЛЛЕР ДЛЯ РОБОТА
Робот едет вперед, при обнаружении препятствия - поворачивает
"""

from controller import Robot
import time

print("=" * 50)
print("ЗАПУСК ПРОСТОГО РОБОТА")
print("=" * 50)

# Создаем робота
robot = Robot()

# Устанавливаем время шага симуляции
timestep = 64  # миллисекунды

# Получаем двигатели
left_motor = robot.getDevice('left_motor')
right_motor = robot.getDevice('right_motor')

# Настраиваем двигатели
left_motor.setPosition(float('inf'))  # бесконечное вращение
right_motor.setPosition(float('inf'))

# Начальная скорость = 0
left_motor.setVelocity(0.0)
right_motor.setVelocity(0.0)

# Получаем датчик расстояния
distance_sensor = robot.getDevice('dsensor')
distance_sensor.enable(timestep)  # включаем датчик

# Параметры управления
MAX_SPEED = 6.28  # максимальная скорость
OBSTACLE_DISTANCE = 100  # расстояние до препятствия для реакции

# Переменные состояния
state = "ВПЕРЕД"  # начальное состояние
turn_start_time = 0
turn_direction = ""

print("Робот готов к работе!")
print("Пресс 'Run simulation' для запуска")
print("-" * 30)

# Ждем 2 секунды перед стартом
print("Старт через 2 секунды...")
start_delay = robot.getTime()
while robot.getTime() - start_delay < 2.0:
    robot.step(timestep)

print("ПОЕХАЛИ!")
print("-" * 30)

# Основной цикл управления
while robot.step(timestep) != -1:
    # Получаем текущее время
    current_time = robot.getTime()
    
    # Читаем значение с датчика расстояния
    distance_value = distance_sensor.getValue()
    
    # Выводим информацию каждые 0.5 секунды
    if int(current_time * 2) % 2 == 0:
        print(f"[{current_time:.1f}с] Датчик: {distance_value:5.1f} | Состояние: {state}")
    
    # АВТОМАТИЧЕСКОЕ УПРАВЛЕНИЕ
    if state == "ВПЕРЕД":
        # ЕДЕМ ВПЕРЕД
        left_motor.setVelocity(0.5 * MAX_SPEED)
        right_motor.setVelocity(0.5 * MAX_SPEED)
        
        # Проверяем, есть ли препятствие
        if distance_value > OBSTACLE_DISTANCE:
            print(f"⚠️  ОБНАРУЖЕНО ПРЕПЯТСТВИЕ! ({distance_value:.1f})")
            print("   Поворачиваю...")
            
            # Выбираем направление поворота
            # Если четная секунда - налево, нечетная - направо
            if int(current_time) % 2 == 0:
                turn_direction = "НАЛЕВО"
                state = "ПОВОРОТ_НАЛЕВО"
            else:
                turn_direction = "НАПРАВО"
                state = "ПОВОРОТ_НАПРАВО"
            
            turn_start_time = current_time
    
    elif state == "ПОВОРОТ_НАЛЕВО":
        # ПОВОРАЧИВАЕМ НАЛЕВО
        left_motor.setVelocity(-0.3 * MAX_SPEED)   # левое колесо назад
        right_motor.setVelocity(0.7 * MAX_SPEED)   # правое колесо вперед
        
        # Проверяем, закончился ли поворот
        if current_time - turn_start_time > 1.0:  # поворачиваем 1 секунду
            print("   Поворот налево завершен!")
            state = "ВПЕРЕД"
    
    elif state == "ПОВОРОТ_НАПРАВО":
        # ПОВОРАЧИВАЕМ НАПРАВО
        left_motor.setVelocity(0.7 * MAX_SPEED)    # левое колесо вперед
        right_motor.setVelocity(-0.3 * MAX_SPEED)  # правое колесо назад
        
        # Проверяем, закончился ли поворот
        if current_time - turn_start_time > 1.0:  # поворачиваем 1 секунду
            print("   Поворот направо завершен!")
            state = "ВПЕРЕД"

print("-" * 30)
print("СИМУЛЯЦИЯ ЗАВЕРШЕНА")
print("=" * 50)