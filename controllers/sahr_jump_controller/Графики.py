"""Построение графиков для SAHR Controller.
Отображает: высоту, целевой/фактический угол ноги, контакт с землёй,
           и траекторию поворота вокруг столба (вид сверху).
"""

import matplotlib.pyplot as plt
import numpy as np


def read_csv(filename):
    """Чтение CSV-файла, созданного контроллером SAHR."""
    data = {
        'time': [], 'phase': [], 'z': [], 'x': [],
        'compression': [], 'leg_length': [],
        'hip_angle': [], 'target_hip_angle': [],
        'vx': [], 'vy': [], 'omega': [], 'contact': [],
        'arm_angle': []
    }

    with open(filename, 'r') as f:
        lines = f.readlines()

    if len(lines) < 2:
        print("Ошибка: файл пуст или содержит только заголовки")
        return None

    for line in lines[1:]:
        if not line.strip():
            continue
        vals = line.strip().split(',')
        if len(vals) < 13:
            continue

        data['time'].append(float(vals[0]))
        data['phase'].append(vals[1])
        data['z'].append(float(vals[2]))
        data['x'].append(float(vals[3]))
        data['compression'].append(float(vals[4]))
        data['leg_length'].append(float(vals[5]))
        data['hip_angle'].append(float(vals[6]))
        data['target_hip_angle'].append(float(vals[7]))
        data['vx'].append(float(vals[8]))
        data['vy'].append(float(vals[9]))
        data['omega'].append(float(vals[10]))
        data['contact'].append(float(vals[11]))
        data['arm_angle'].append(float(vals[12]))

    return data


def add_phase_background(ax, time, phases):
    """Добавляет цветовой фон по фазам."""
    phase_colors = {
        'START': '#e0e0e0',
        'SPINUP': '#ffcc80',
        'BRAKE_JUMP': '#ff8a80',
        'FLIGHT': '#80cbc4',
        'PRELAND': '#fff59d',
        'LANDING': '#90caf9'
    }

    if len(time) < 2:
        return

    current_phase = phases[0]
    phase_start = time[0]

    for i in range(1, len(time)):
        if phases[i] != current_phase:
            color = phase_colors.get(current_phase, '#ffffff')
            ax.axvspan(phase_start, time[i-1], alpha=0.15, color=color, zorder=0)
            phase_start = time[i]
            current_phase = phases[i]

    color = phase_colors.get(current_phase, '#ffffff')
    ax.axvspan(phase_start, time[-1], alpha=0.15, color=color, zorder=0)

    from matplotlib.patches import Patch
    unique_phases = list(dict.fromkeys(phases))
    legend_elements = [Patch(facecolor=phase_colors.get(p, '#ffffff'), alpha=0.3, label=p)
                       for p in unique_phases if p in phase_colors]
    ax.legend(handles=legend_elements, loc='upper right', fontsize=7, ncol=3)


def plot_sahr_data(csv_file, save_fig=True):
    """Основная функция построения графиков."""
    data = read_csv(csv_file)
    if data is None:
        return

    time = np.array(data['time'])
    z = np.array(data['z'])
    hip_angle_deg = np.degrees(data['hip_angle'])
    target_hip_angle_deg = np.degrees(data['target_hip_angle'])
    contact = np.array(data['contact'])
    arm_angle_rad = np.array(data['arm_angle'])  # угол в радианах
    phases = data['phase']

    # Уровень земли
    ground_mask = contact > 0.5
    ground_level = np.median(z[ground_mask]) if np.any(ground_mask) else np.min(z)
    height = z - ground_level

    # Создаём фигуру с сеткой 2x2 (4 подграфика)
    fig = plt.figure(figsize=(16, 12))
    fig.suptitle('Эксперимент №2: Управление ногой робота для сохранения энергии прыжка',
                 fontsize=14, fontweight='bold')

    # === 1. ВЫСОТА (верхний левый) ===
    ax1 = plt.subplot(2, 2, 1)
    add_phase_background(ax1, time, phases)
    ax1.plot(time, height, 'b-', linewidth=1.2, label='Высота над землёй')
    ax1.axhline(y=0, color='k', linestyle='--', alpha=0.5, label='Уровень земли')
    ax1.axhline(y=height[0], color='green', linestyle=':', alpha=0.5, label=f'Начальная: {height[0]:.3f} м')
    ax1.axhline(y=height[-1], color='red', linestyle=':', alpha=0.7, label=f'Итоговая: {height[-1]:.3f} м')
    ax1.set_ylabel('Высота, м')
    ax1.set_xlabel('Время, с')
    ax1.grid(True, alpha=0.3)
    ax1.legend(loc='upper left', fontsize=8)
    ax1.set_title('Высота корпуса робота')

    # === 2. УГОЛ НОГИ (верхний правый) ===
    ax2 = plt.subplot(2, 2, 2)
    add_phase_background(ax2, time, phases)
    ax2.plot(time, target_hip_angle_deg, 'r--', linewidth=1.5, label='Целевой угол φ_ref')
    ax2.plot(time, hip_angle_deg, 'b-', linewidth=1.2, label='Фактический угол φ')
    ax2.fill_between(time, hip_angle_deg, target_hip_angle_deg, alpha=0.2, color='red', label='Ошибка')
    ax2.set_ylabel('Угол ноги φ, град')
    ax2.set_xlabel('Время, с')
    ax2.grid(True, alpha=0.3)
    ax2.legend(loc='upper left', fontsize=8)
    ax2.set_title('Угол постановки ноги: желаемый vs фактический')

    # === 3. КОНТАКТ С ЗЕМЛЁЙ (нижний левый) ===
    ax3 = plt.subplot(2, 2, 3)
    add_phase_background(ax3, time, phases)
    ax3.plot(time, contact, 'purple', linewidth=1.5, drawstyle='steps-post', label='Контакт')
    ax3.fill_between(time, 0, contact, alpha=0.25, color='purple', step='post')
    ax3.set_ylabel('Контакт (0/1)')
    ax3.set_xlabel('Время, с')
    ax3.set_ylim([-0.1, 1.1])
    ax3.grid(True, alpha=0.3)
    ax3.legend(loc='upper left', fontsize=8)
    ax3.set_title('Контакт стопы с поверхностью')

    # === 4. ПОЛЯРНЫЙ ГРАФИК: траектория вокруг стойки (нижний правый) ===
    ax4 = plt.subplot(2, 2, 4, projection='polar')
    
    # Преобразуем углы в радианы (если ещё не в радианах)
    # arm_angle_rad уже в радианах из датчика
    
    # Добавляем первую точку в конец для замыкания траектории (опционально)
    angles = arm_angle_rad
    # Используем фиксированный радиус = 1 (вид сверху, важны только углы)
    radii = np.ones_like(angles)
    
    # Рисуем траекторию: линия, соединяющая последовательные положения
    ax4.plot(angles, radii, 'g-', linewidth=1.5, alpha=0.7, label='Траектория движения')
    
    # Отмечаем начальную точку
    ax4.plot(angles[0], 1, 'go', markersize=8, label='Старт')
    
    # Отмечаем конечную точку
    ax4.plot(angles[-1], 1, 'ro', markersize=8, label='Финиш')
    
    # Отмечаем точки отрыва (где контакт = 0 → 1)
    lift_off_indices = []
    for i in range(1, len(contact)):
        if contact[i] == 0 and contact[i-1] == 1:
            lift_off_indices.append(i)
    
    for idx in lift_off_indices:
        ax4.plot(angles[idx], 1, 'b^', markersize=6, alpha=0.7)
    
    # Настройка полярного графика
    ax4.set_theta_zero_location('N')      # 0° — вверх (север)
    ax4.set_theta_direction(-1)           # положительное направление — по часовой стрелке
    ax4.set_ylim(0, 1.2)                  # радиус от 0 до 1.2 (фиксирован для вида сверху)
    ax4.set_yticks([])                    # скрываем метки радиуса (они не нужны)
    ax4.set_title('Траектория движения вокруг стойки (вид сверху)', va='bottom')
    
    # Легенда для полярного графика
    from matplotlib.lines import Line2D
    legend_elements_polar = [
        Line2D([0], [0], color='g', linewidth=1.5, label='Траектория'),
        Line2D([0], [0], marker='o', color='w', markerfacecolor='g', markersize=8, label='Старт'),
        Line2D([0], [0], marker='o', color='w', markerfacecolor='r', markersize=8, label='Финиш'),
        Line2D([0], [0], marker='^', color='w', markerfacecolor='b', markersize=6, label='Отрыв')
    ]
    ax4.legend(handles=legend_elements_polar, loc='upper right', fontsize=7)
    
    plt.tight_layout()

    if save_fig:
        plt.savefig('sahr_analysis.png', dpi=150, bbox_inches='tight')
        print("График сохранён: sahr_analysis.png")

    plt.show()
    print_statistics(time, height, hip_angle_deg, target_hip_angle_deg, contact, arm_angle_rad)


def print_statistics(time, height, hip_deg, target_deg, contact, arm_rad):
    """Вывод числовой статистики эксперимента."""
    arm_deg = np.degrees(arm_rad)
    
    print("\n" + "=" * 60)
    print("СТАТИСТИКА ЭКСПЕРИМЕНТА")
    print("=" * 60)

    print(f"\nВРЕМЯ:")
    print(f"  Длительность: {time[-1]:.2f} с")
    print(f"  Шагов: {len(time)}")

    print(f"\nВЫСОТА:")
    print(f"  Начальная: {height[0]:.3f} м")
    print(f"  Максимальная: {np.max(height):.3f} м")
    print(f"  Минимальная: {np.min(height):.3f} м")
    print(f"  Итоговая: {height[-1]:.3f} м")

    print(f"\nУГОЛ НОГИ:")
    print(f"  Начальный целевой: {target_deg[0]:.1f}°")
    print(f"  Начальный фактический: {hip_deg[0]:.1f}°")
    print(f"  Макс. ошибка: {np.max(np.abs(hip_deg - target_deg)):.1f}°")
    print(f"  Средняя ошибка: {np.mean(np.abs(hip_deg - target_deg)):.1f}°")

    print(f"\nУГОЛ ПОВОРОТА ВОКРУГ СТОЛБА:")
    print(f"  Начальный: {arm_deg[0]:.1f}°")
    print(f"  Конечный: {arm_deg[-1]:.1f}°")
    print(f"  Суммарный поворот: {arm_deg[-1] - arm_deg[0]:.1f}°")
    print(f"  Эквивалентно: {(arm_deg[-1] - arm_deg[0]) / 360:.2f} оборота(ов)")

    print(f"\nКОНТАКТ:")
    contact_pct = np.sum(contact > 0.5) / len(contact) * 100
    print(f"  Время в контакте: {contact_pct:.1f}%")
    print(f"  Время в полёте: {100 - contact_pct:.1f}%")

    lift_off_count = sum(1 for i in range(1, len(contact)) if contact[i] == 0 and contact[i-1] == 1)
    print(f"  Количество отрывов: {lift_off_count}")

    print("\n" + "=" * 60)


if __name__ == "__main__":
    plot_sahr_data("sahr_data.csv", save_fig=True)
