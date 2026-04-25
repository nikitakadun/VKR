"""Построение графиков для эксперимента №1.
С подписью итоговых значений на каждом графике.
"""

import matplotlib.pyplot as plt
import re
import math

ARM_LENGTH = 0.45  # метры

def read_csv_simple(filename):
    with open(filename, 'r') as f:
        lines = f.readlines()
    if not lines:
        return {}, []
    headers = lines[0].strip().split(',')
    data = {h: [] for h in headers}
    for line in lines[1:]:
        if not line.strip():
            continue
        vals = line.strip().split(',')
        for i, h in enumerate(headers):
            if i < len(vals):
                try:
                    data[h].append(float(vals[i]))
                except ValueError:
                    data[h].append(0.0)
    return data, headers

def plot_experiment(csv_file, save_fig=False):
    data, headers = read_csv_simple(csv_file)
    if not data or not data.get('time'):
        print(f"Ошибка: нет данных в {csv_file}")
        return
    
    time = data['time']
    match = re.search(r'angle_([+-]?\d+)', csv_file)
    angle = match.group(1) if match else "?"
    
    fig, axes = plt.subplots(5, 1, figsize=(12, 13))
    fig.suptitle(f'Эксперимент №1: Пассивное падение с углом ноги 0°', fontsize=14)
    
    # --- 1. ВЫСОТА ---
    if 'height_from_gps' in data:
        height_col = 'height_from_gps'
    elif 'gps_z' in data:
        height_col = 'gps_z'
    elif 'height' in data:
        height_col = 'height'
    else:
        height_col = None
    
    if height_col:
        height_data = data[height_col]
        axes[0].plot(time, height_data, 'b-', linewidth=1.5)
        axes[0].set_ylabel('Высота, м')
        axes[0].set_xlabel('Время, с')
        axes[0].grid(True, alpha=0.3)
        axes[0].axhline(y=0, color='k', linestyle='--', alpha=0.5, label='Уровень земли')
        
        # Итоговое значение высоты
        final_height = height_data[-1]
        axes[0].axhline(y=final_height, color='r', linestyle=':', alpha=0.7, 
                       label=f'Итоговая высота: {final_height:.3f} м')
        
        y_max = max(height_data)
        y_min = min(min(height_data), -0.05)
        axes[0].set_ylim([y_min - 0.05, y_max + 0.05])
        axes[0].legend()
    
    # --- 2. УГОЛ НОГИ ---
    if 'hip_angle_deg' in data:
        angle_data = data['hip_angle_deg']
        initial_angle = angle_data[0] if len(angle_data) > 0 else 0
        final_angle = angle_data[-1] if len(angle_data) > 0 else 0
        
        axes[1].plot(time, angle_data, 'g-', linewidth=1.5)
        axes[1].set_ylabel('Угол ноги φ, град')
        axes[1].set_xlabel('Время, с')
        axes[1].grid(True, alpha=0.3)
        axes[1].axhline(y=initial_angle, color='k', linestyle='--', alpha=0.5, 
                       label=f'Начальный угол: {initial_angle:.1f}°')
        axes[1].axhline(y=final_angle, color='r', linestyle=':', alpha=0.7,
                       label=f'Итоговый угол: {final_angle:.1f}°')
        
        y_min = min(angle_data)
        y_max = max(angle_data)
        margin = max(5, (y_max - y_min) * 0.1)
        axes[1].set_ylim([y_min - margin, y_max + margin])
        axes[1].legend()
    
    # --- 3. ДЛИНА НОГИ ---
    if 'leg_length' in data:
        leg_data = data['leg_length']
        nominal_leg = 0.1
        final_leg = leg_data[-1]
        
        axes[2].plot(time, leg_data, 'r-', linewidth=1.5)
        axes[2].set_ylabel('Длина ноги r, м')
        axes[2].set_xlabel('Время, с')
        axes[2].grid(True, alpha=0.3)
        axes[2].axhline(y=nominal_leg, color='k', linestyle='--', alpha=0.5, 
                       label=f'Номинальная длина: {nominal_leg:.3f} м')
        axes[2].axhline(y=final_leg, color='r', linestyle=':', alpha=0.7,
                       label=f'Итоговая длина: {final_leg:.3f} м')
        
        y_min = min(leg_data)
        y_max = max(leg_data)
        margin = max(0.01, (y_max - y_min) * 0.1)
        axes[2].set_ylim([y_min - margin, y_max + margin])
        axes[2].legend()
    
    # --- 4. КОНТАКТ ---
    contact_col = None
    for col in ['contact', 'foot_force']:
        if col in data:
            contact_col = col
            break
    
    if contact_col:
        contact_data = data[contact_col]
        axes[3].plot(time, contact_data, 'purple', linewidth=1, drawstyle='steps-post')
        axes[3].set_ylabel('Контакт (0/1)')
        axes[3].set_xlabel('Время, с')
        axes[3].set_ylim([-0.1, 1.1])
        axes[3].grid(True, alpha=0.3)
        axes[3].fill_between(time, 0, contact_data, alpha=0.3, color='purple')
        
        # Добавляем подпись с долей времени контакта
    
    # --- 5. ГОРИЗОНТАЛЬНОЕ ПЕРЕМЕЩЕНИЕ X ---
    if 'x_displacement' in data:
        x_data = data['x_displacement']
        final_x = x_data[-1]
        max_x = max(x_data)
        
        axes[4].plot(time, x_data, 'c-', linewidth=1.5)
        axes[4].set_ylabel('Перемещение X, м')
        axes[4].set_xlabel('Время, с')
        axes[4].grid(True, alpha=0.3)
        axes[4].axhline(y=0, color='k', linestyle='--', alpha=0.5)
        axes[4].axhline(y=final_x, color='r', linestyle=':', alpha=0.7,
                       label=f'Итоговое перемещение: {final_x:.4f} м ({final_x*100:.2f} см)')
        axes[4].set_title('Горизонтальное перемещение')
        
        # Масштабирование от 0 до максимума
        y_upper = max_x + max(0.005, max_x * 0.1)
        axes[4].set_ylim([0, y_upper])
        axes[4].legend()
        
        print(f"\n=== Перемещение X ===")
        print(f"Максимум: {max_x:.4f} м ({max_x*100:.2f} см)")
        print(f"Итоговое: {final_x:.4f} м ({final_x*100:.2f} см)")
        
    elif 'arm_angle_deg' in data:
        arm_angle_rad = [math.radians(deg) for deg in data['arm_angle_deg']]
        x_displacement = [ARM_LENGTH * math.sin(rad) for rad in arm_angle_rad]
        final_x = x_displacement[-1]
        max_x = max(x_displacement)
        
        axes[4].plot(time, x_displacement, 'c-', linewidth=1.5)
        axes[4].set_ylabel('Перемещение X, м')
        axes[4].set_xlabel('Время, с')
        axes[4].grid(True, alpha=0.3)
        axes[4].axhline(y=0, color='k', linestyle='--', alpha=0.5)
        axes[4].axhline(y=final_x, color='r', linestyle=':', alpha=0.7,
                       label=f'Итоговое перемещение: {final_x:.4f} м ({final_x*100:.2f} см)')
        axes[4].set_title(f'Горизонтальное перемещение (R={ARM_LENGTH} м)')
        
        y_upper = max_x + max(0.005, max_x * 0.1)
        axes[4].set_ylim([0, y_upper])
        axes[4].legend()
        
        print(f"\n=== Перемещение X ===")
        print(f"Максимум: {max_x:.4f} м ({max_x*100:.2f} см)")
        print(f"Итоговое: {final_x:.4f} м ({final_x*100:.2f} см)")
    
    plt.tight_layout()
    if save_fig:
        plt.savefig(f'plot_angle_{angle}.png', dpi=150)
    plt.show()


def print_full_statistics(csv_file):
    """Вывод полной статистики по эксперименту."""
    data, headers = read_csv_simple(csv_file)
    if not data:
        return
    
    print("\n" + "="*60)
    print("ПОЛНАЯ СТАТИСТИКА ЭКСПЕРИМЕНТА")
    print("="*60)
    
    # Высота
    for h in ['height_from_gps', 'gps_z', 'height']:
        if h in data:
            print(f"\n📊 ВЫСОТА:")
            print(f"   Начальная: {data[h][0]:.3f} м")
            print(f"   Максимальная: {max(data[h]):.3f} м")
            print(f"   Минимальная: {min(data[h]):.3f} м")
            print(f"   Итоговая: {data[h][-1]:.3f} м")
            break
    
    # Угол ноги
    if 'hip_angle_deg' in data:
        print(f"\n📊 УГОЛ НОГИ:")
        print(f"   Начальный: {data['hip_angle_deg'][0]:.1f}°")
        print(f"   Максимальный: {max(data['hip_angle_deg']):.1f}°")
        print(f"   Минимальный: {min(data['hip_angle_deg']):.1f}°")
        print(f"   Итоговый: {data['hip_angle_deg'][-1]:.1f}°")
    
    # Длина ноги
    if 'leg_length' in data:
        print(f"\n📊 ДЛИНА НОГИ:")
        print(f"   Номинальная: 0.100 м")
        print(f"   Минимальная (сжатие): {min(data['leg_length']):.3f} м")
        print(f"   Итоговая: {data['leg_length'][-1]:.3f} м")
    
    # Перемещение
    if 'x_displacement' in data:
        print(f"\n📊 ПЕРЕМЕЩЕНИЕ:")
        print(f"   Максимальное: {max(data['x_displacement'])*100:.2f} см")
        print(f"   Итоговое: {data['x_displacement'][-1]*100:.2f} см")
    elif 'arm_angle_deg' in data:
        final_rad = math.radians(data['arm_angle_deg'][-1])
        print(f"\n📊 ПЕРЕМЕЩЕНИЕ:")
        print(f"   Итоговый угол рычага: {data['arm_angle_deg'][-1]:.1f}°")
        print(f"   Итоговое: {ARM_LENGTH * math.sin(final_rad)*100:.2f} см")
    
    # Контакт
    for col in ['contact', 'foot_force']:
        if col in data:
            contact_pct = sum(data[col]) / len(data[col]) * 100
            print(f"\n📊 КОНТАКТ С ЗЕМЛЁЙ:")
            print(f"   Доля времени: {contact_pct:.1f}%")
            break
    
    print("\n" + "="*60)


if __name__ == "__main__":
    CSV_FILE = "experiment_fall_angle_+0.csv"
    
    plot_experiment(CSV_FILE, save_fig=True)
    print_full_statistics(CSV_FILE)
