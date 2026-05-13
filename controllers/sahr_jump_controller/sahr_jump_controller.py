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
R_ARM           = 0.45          # радиус рычага (оценка по геометрии стойки), м
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
KP_HIP           = 400.0       # ПД‑регулятор угла бедра
KD_HIP           = 12.0
GR_TORQUE_MAX    = 100.0       # макс. момент груза (совпадает с maxTorque в сцене), Н·м
HIP_TORQUE_MAX   = 100.0       # макс. момент бедра (совпадает с maxTorque в сцене), Н·м
BRAKE_TORQUE     = -100.0      # тормозной момент (равен -GR_TORQUE_MAX)
BRAKE_DURATION   = 0.03        # длительность торможения, с
LANDING_HOLD     = 0.3         # время удержания после касания, с
FULL_TURN_RAD    = 2 * math.pi
ANGLE_TOLERANCE  = 0.05        # допуск при определении завершения оборота, рад

# ---------- Целевые показатели ----------
TARGET_HEIGHT  = 0.35          # желаемая высота прыжка, м
TARGET_SPEED_X = 0.5          # желаемая тангенциальная скорость, м/с

# ---------- Коэффициенты верхнего уровня ----------
KP_HEIGHT_ERR    = 20.0        # добавочная скорость груза на метр ошибки по высоте
KP_SPEED_ERR     = 0.2 

# Желаемое значение энергии для прыжка
E_DES        = M_TOTAL * G * TARGET_HEIGHT + 1/2 * M_TOTAL * TARGET_SPEED_X ** 2
# Желаемый угол прыжка
PHI_JUMP_DES = -math.atan(TARGET_SPEED_X / math.sqrt(2 * G * TARGET_HEIGHT))
# Желаемая скорость вращения груза
OMEGA_DES    = math.sqrt(2 * E_DES / (ETA * I_GR))# √(2·E_des / (η·I))

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
        self.arm_angle_sensor = self.robot.getDevice("arm_angle_sensor")
        self.arm_angle_sensor.enable(TIME_STEP)
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
        self.vz = 0.0
        self.phase_start_time = 0.0
        self.brake_start_time = None
        self.prev_hip_error = 0.0
        self.prev_z = None
        self.prev_compression = None

        self.current_E = 0
        
        self.permission = False
        
        # Переменные для SAHR-кинематики
        self.prev_leg_len = NOMINAL_LEG_LENGTH
        self.leg_velocity = 0.0          # r˙ – скорость изменения длины ноги
        self.hip_angular_velocity = 0.0  # φ˙ – угловая скорость бедра
        self.prev_hip_angle = 0.0
        self.vx_stance = 0.0             # x˙ в фазе опоры
        self.vz_stance = 0.0             # y˙ в фазе опоры
                

        # Целевые уставки
        self.target_spin_speed = 0.0
        self.target_hip_angle = -0.244346 # JUMP_HIP_ANGLE

        self.target_delta_r = 0

        self.target_E = E_DES
        self.reserve_E = None
        self.additional_E = 0

        # Создание CSV-файла для записи данных
        self.csv_file = open("sahr_data.csv", "w", newline="")
        self.csv_writer = csv.writer(self.csv_file)
        
        # Заголовки столбцов
        self.csv_writer.writerow([
            "time", "phase", 
            "z", "x", 
            "compression", "leg_length",
            "hip_angle", "target_hip_angle",
            "vx", "vz", 
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
        arm_angle = self.arm_angle_sensor.getValue() if self.arm_angle_sensor else 0.0
        return {'x': gps[0], 'z': gps[2], 'contact': contact, 'leg_len': leg_len,
                'compression': compression, 'cent_angle': cent_angle, 'hip_angle': hip_angle, 'arm_angle': arm_angle}

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

    """
    !!!!!! к удалению !!!!!
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
    """
    ### уточнить нужно ли это удалить
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
        self.vz =  r_dot * math.cos(phi) - r * phi_dot * math.sin(phi)

    # ========== Горизонтальная скорость (GPS) ==========
    def compute_vx(self, s):
        dt = TIME_STEP / 1000.0
        self.vx = (s['x'] - self.prev_x) / dt
        self.prev_x = s['x']

    def compute_vz_no_contact(self, s):
        dt = TIME_STEP / 1000.0
        self.vz = (s['z'] - self.prev_z ) / dt
        self.prev_z = s['z']

    # ========== Расчёт угла приземления ==========
    def compute_touchdown_angle(self):
    # Если скорость слишком мала — используем базовый угол
        if abs(self.vx) < 0.01 and abs(self.vz) < 0.01:
            return PHI_JUMP_DES
        
        # Угол вектора скорости относительно вертикали
        # vz < 0 при падении (робот снижается)
        
        ### !!!!!!!!!!!! правильно тут считается вообще?
        if self.vz < 0:
            #angle = -math.atan2(self.vx, -self.vz)
            angle = -PHI_JUMP_DES
            return max(0, min(0.2619, angle))
        else:
            # Если z положительна (взлетает) — используем базовый угол
            angle = PHI_JUMP_DES
        
        # Ограничиваем угол в разумных пределах (-34° до +15°)
        return max(-0.6, min(0.2619, angle))
        
    ### скорее всего тоже удалим    
    # ========== Расчёт угла прыжка ==========
    def compute_jump_angle(self):
        self.target_hip_angle = math.atan(TARGET_SPEED_X / math.sqrt(2 * G * TARGET_HEIGHT)) #arctan(v_x_des / √(2·g·h_des))
    
    
        # к удалению
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
        
        # к удалению
    # ========== Расчёт желаемой энергии ==========
    def calculate_target_energy(self, current_E, current_v):
        """
        Желаемая энергия -- это значение нынешней желаемой энергии - значение запасенной энергии
        Первый прыжок считаем по желаемой энергии по дефолту
        Для последующих прыжков считаем по формуле Еж = Ен - Ез
        Это нужно считать в момент максимального сжатия пружины
        """
        pass 

    def recalculation(self, s):
        dt = TIME_STEP / 1000.0

        if self.start_angle is None:
            self.start_angle = s['cent_angle']
        delta_r = s['compression']
        if self.prev_compression == None:
            self.prev_compression = delta_r
        if self.phase == SPINUP:
            if (delta_r - self.prev_compression) / dt < 0:
                self.permission = True
                if self.reserve_E == None:
                    self.reserve_E = 0.5 * K_SPRING * self.prev_compression ** 2 + 0.5 * M_TOTAL * self.vx 
                self.additional_E = E_DES - self.reserve_E
                self.additional_v = math.sqrt(2 * self.additional_E / M_TOTAL)
                self.additional_p = M_TOTAL * self.additional_v
                self.current_E = 0.5 * K_SPRING * self.prev_compression ** 2 + 0.5 * M_TOTAL * self.vx + 0.5 * self.omega * I_GR
                #print(f'reserve E = {self.reserve_E} | additional E = {self.additional_E} | E_DES = {E_DES}'
                #      f' | current E = {self.current_E}')
                #self.target_E = E_DES - self.current_E

                    # Желаемый угол прыжка
                self.target_hip_angle  = -math.atan(TARGET_SPEED_X / math.sqrt(2 * G * TARGET_HEIGHT))
                    # Желаемая скорость вращения груза
                #self.target_spin_speed = math.sqrt(self.additional_p * R_ARM / (ETA * I_GR))
                self.target_spin_speed = math.sqrt(self.additional_E * 2 / (ETA * I_GR))
        
        if self.phase == FLIGHT:
            if s['cent_angle'] % (2 * math.pi) >= 6.08:
                self.permission = False
               
        self.prev_compression = delta_r


    # это надо?
    def calculate_current_energy(self, s):
        """
        Рассчитывает текущую полную энергию системы.
        s — словарь с показаниями датчиков.
        Возвращает E_total в Джоулях.
        """
        # 1. Кинетическая энергия вращения груза
        E_rot = 0.5 * I_GR * self.omega ** 2
        
        # 2. Потенциальная энергия сжатой пружины
        # compression = NOMINAL_LEG_LENGTH - leg_len, положительно при сжатии
        compression = s['compression']
        # Пружина имеет энергию только когда сжата (compression > 0)
        if compression > 0:
            E_spring = 0.5 * K_SPRING * compression ** 2
        else:
            E_spring = 0.0
        
        # 3. Кинетическая энергия поступательного движения корпуса
        # vx из compute_velocity, vz из compute_vz_no_contact или из GPS
        # Обратите внимание: в коде vz вычисляется только в полёте.
        # Для полной картины лучше брать вертикальную скорость из GPS: 
        # gps_velocity = s.get('vz_from_gps', 0) или использовать self.vz
        E_trans = 0.5 * M_TOTAL * (self.vx ** 2 + self.vz ** 2)
        
        E_total = E_rot + E_spring + E_trans
 #       print(f'E_rot = {E_rot} | E_trans = {E_trans} | E_spring = {E_spring} | E_total = {E_total}')
        self.current_E = E_total  

    def update_phases(self, s):
        contact = s['contact']
        #self.centrifugal_motor.setTorque(5)
        
        if self.phase == START:
            if contact:
                #self.target_spin_speed, self.target_E = OMEGA_DES, E_DES #self.calculate_setpoints(TARGET_HEIGHT)
                self.target_spin_speed, self.target_hip_angle = OMEGA_DES, PHI_JUMP_DES
                print(f"Target spin: {self.target_spin_speed:.1f} rad/s, Δr: {self.target_E*1000:.1f} J")
                self._set_phase(SPINUP)

        elif not contact and self.phase != FLIGHT:
            if self.vz > 0:
                self._set_phase(FLIGHT)
       
        elif self.phase == FLIGHT:
            #h = s['z'] - self.ground_z if self.ground_z else 0
            #print(f'vz = {self.vz}')
            
            if self.vz < 0:
                #self.target_hip_angle = self.compute_touchdown_angle()
                self._set_phase(LANDING)

        elif self.phase == SPINUP:
            """
            h_des и v_des напрямую зависимы от vx vz, которые задаются
            в compute_stance_velocity
            """
            angle = s['hip_angle']
            if ((self.current_E <= E_DES + 5 and 
                self.current_E  >= E_DES - 5) and
                (angle <= self.target_hip_angle + 0.018 and
                 angle >= self.target_hip_angle - 0.018)):
                self._set_phase(FLIGHT)
                
        
        elif self.phase == LANDING:
            self.permission = False
            if contact:
                self.reserve_E = None
                self._set_phase(SPINUP)

    # ========== Расчет скорости для положения груза в 15 * =======
    def hold_spin_speed(self):
        hold_speed = 0
        
        return hold_speed


    # ========== Управление центробежным грузом ==========
    def control_centrifugal(self):
        t = self.robot.getTime()
        if self.permission:
            err = self.target_spin_speed - self.omega
            
        else:
            err = 0 - self.omega # Груз должен удерживаться на месте
        tau = KP_SPIN * err   
        tau = max(-GR_TORQUE_MAX, min(GR_TORQUE_MAX, tau))

        #print(f'tau = {tau} | target omega = {self.target_spin_speed} | omega = {self.omega}')
        self.centrifugal_motor.setTorque(tau)    

    # ========== Управление бедром ==========
    def control_hip(self):
        if self.phase == START:
            self.hip_motor.setTorque(0.0)
            return
        current = self.hip_sensor.getValue() if self.hip_sensor else 0.0
        if self.phase == SPINUP or self.phase == FLIGHT:
            self.target_hip_angle = PHI_JUMP_DES
        if self.phase == LANDING:
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
            f"{self.vz:.6f}",
            f"{self.omega:.6f}",
            f"{1 if s['contact'] else 0}",
            f"{s['arm_angle']:.6f}"
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
                      f"vx_s={self.vx_stance:.2f} | vz_s={self.vz_stance:.2f} | "
                      f"ω={self.omega:.1f}")
            else:
                print(f"t={t:.2f}s | {PHASE_NAMES[self.phase]} | "
                      f"h={h:.3f}m | ω={self.omega:.1f} | φ_ref={self.target_hip_angle:.3f} | "
                      f"φ={s['hip_angle']:.2f} | vx={self.vx:.2f}")

    def run(self):
        while self.robot.step(TIME_STEP) != -1:
            s = self.read_sensors()
            angle = s['cent_angle']
            print(f'cent angle = {angle}')
            self.compute_omega(s)
            self.compute_vx(s)
            #self.calculate_current_energy(s)
            self.recalculation(s)
            #if self.phase == SPINUP:
                #self.compute_stance_kinematics(s)
                #self.compute_stance_velocity(s)
            if not s['contact']:
                self.compute_vz_no_contact(s)
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