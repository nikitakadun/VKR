"""SAHR Controller – energy-saving hopping with adaptive touchdown angle."""

from controller import Robot
import math

TIME_STEP = 1
NOMINAL_LEG_LENGTH = 0.1

( START, SPINUP, LIFTOFF, FLIGHT, PRELAND, LANDING ) = range(6)
PHASE_NAMES = ['START','SPINUP','LIFTOFF','FLIGHT','PRELAND','LANDING']

# ---------- Целевые показатели ----------
TARGET_HEIGHT  = 0.10          # м
TARGET_SPEED_X = 0.5           # м/с

# ---------- Параметры регуляторов ----------
BASE_SPIN_SPEED  = 40.0        # базовая угловая скорость груза, рад/с
BASE_HIP_ANGLE   = -0.2619     # -15° (угол по умолчанию)
JUMP_HIP_ANGLE   = -0.4        # угол в фазе опоры (для следующего прыжка)
KP_SPIN          = 2.0         # П‑регулятор скорости груза
KP_HIP           = 15.0        # ПД‑регулятор угла бедра (усилен)
KD_HIP           = 0.5
GR_TORQUE_MAX    = 100.0
HIP_TORQUE_MAX   = 30.0        # увеличен максимальный момент
MIN_SPINUP_TIME  = 0.4         # с
BRAKE_DURATION   = 0.03        # с
LANDING_HOLD     = 0.3         # с

# Коэффициенты верхнего уровня (подбираются)
KP_HEIGHT_ERR    = 20.0        # добавочная скорость груза на метр ошибки по высоте
KP_SPEED_ERR     = 0.2         # добавочный угол ноги на м/с ошибки по скорости
# ------------------------------------------

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
        self.prev_cent_angle = 0.0
        self.omega = 0.0
        self.ground_z = 0.149        # фиксированная высота земли (по логам)
        self.apex_height = 0.0
        self.apex_reached = False
        self.apex_time = 0.0
        self.prev_x = 0.0
        self.vx = 0.0
        self.vy = 0.0
        self.phase_start_time = 0.0
        self.prev_hip_error = 0.0
        self.brake_start_time = None

        # Переменные верхнего уровня и адаптивного угла
        self.desired_spin_speed = BASE_SPIN_SPEED
        self.desired_hip_angle  = BASE_HIP_ANGLE
        self.touchdown_angle   = BASE_HIP_ANGLE   # угол для приземления (по вектору скорости)
        self.jump_angle        = JUMP_HIP_ANGLE   # угол для фазы опоры

        print("SAHR Controller (adaptive) ready.")

    # ========== Сенсоры ==========
    def read_sensors(self):
        leg_off = self.leg_sensor.getValue() if self.leg_sensor else 0.0
        leg_len = NOMINAL_LEG_LENGTH + leg_off
        gps = self.body_gps.getValues() if self.body_gps else [0,0,0]
        contact = self.foot_sensor.getValue() > 0.5 if self.foot_sensor else False
        cent_angle = self.centrifugal_sensor.getValue() if self.centrifugal_sensor else 0.0
        hip_angle = self.hip_sensor.getValue() if self.hip_sensor else 0.0
        return {'z': gps[2], 'contact': contact, 'leg_len': leg_len,
                'cent_angle': cent_angle, 'hip_angle': hip_angle, 'x': gps[0]}

    # ========== Вычисления ==========
    def compute_omega(self, s):
        dt = TIME_STEP / 1000.0
        d = s['cent_angle'] - self.prev_cent_angle
        if d > math.pi:       
            d -= 2*math.pi
        elif d < -math.pi:    
            d += 2*math.pi
        self.omega = d / dt
        self.prev_cent_angle = s['cent_angle']

    def compute_velocity(self, s):
        dt = TIME_STEP / 1000.0
        self.vx = (s['x'] - self.prev_x) / dt
        self.prev_x = s['x']

    def compute_touchdown_angle(self):
        """Угол приземления по направлению вектора скорости."""
        # Если скорости практически нулевые, используем угол по умолчанию
        if abs(self.vx) < 0.01 and abs(self.vy) < 0.01:
            return BASE_HIP_ANGLE
        # vy должна быть отрицательной (падение), но для надёжности используем abs(vy) в знаменателе
        angle = -math.atan2(self.vx, -self.vy) if self.vy < 0 else 0.0
        # Ограничиваем диапазон: [-0.6, 0.6] рад (~[-34°, 34°])
        return max(-0.6, min(0.6, angle))

    # ========== Верхний уровень ==========
    def high_level_control(self, s):
        """Вычисляет поправки к уставкам на основе ошибок."""
        height_err = TARGET_HEIGHT - self.apex_height
        self.desired_spin_speed = BASE_SPIN_SPEED + KP_HEIGHT_ERR * height_err
        self.desired_spin_speed = max(10.0, min(100.0, self.desired_spin_speed))

        speed_err = TARGET_SPEED_X - self.vx
        # Поправка для будущего угла приземления через изменение базового угла,
        # но не меняем touchdown_angle здесь – он вычисляется по скорости.
        self.desired_hip_angle = BASE_HIP_ANGLE - KP_SPEED_ERR * speed_err
        self.desired_hip_angle = max(-0.6, min(0.6, self.desired_hip_angle))

    # ========== Автомат фаз ==========
    def update_phase(self, s):
        contact = s['contact']
        liftoff = not contact and self.prev_contact
        touchdown = contact and not self.prev_contact
        self.prev_contact = contact
        t = self.robot.getTime()

        if self.phase == START:
            if contact:
                # Уровень земли уже задан жёстко
                self._set_phase(SPINUP)

        elif self.phase == SPINUP:
            # В фазе раскрутки держим прыжковый угол (носок вперёд)
            self.desired_hip_angle = self.jump_angle
            if (t - self.phase_start_time > MIN_SPINUP_TIME and
                self.omega >= self.desired_spin_speed * 0.95):
                self._set_phase(LIFTOFF)
                self.brake_start_time = t

        elif self.phase == LIFTOFF:
            if liftoff:
                self._set_phase(FLIGHT)
                self.apex_height = s['z'] - self.ground_z
                self.apex_reached = False
                self.apex_time = t
            elif t - self.phase_start_time > 0.2:
                # Если не взлетели – возвращаемся на раскрутку
                self._set_phase(SPINUP)

        elif self.phase == FLIGHT:
            h = s['z'] - self.ground_z
            if h > self.apex_height:
                self.apex_height = h
                self.apex_time = t
            else:
                self.apex_reached = True

            # Вычисляем вертикальную скорость после апогея
            if self.apex_reached:
                dt_apex = t - self.apex_time
                self.vy = -9.81 * dt_apex   # свободное падение
                # Когда скорость известна, вычисляем идеальный угол приземления
                self.touchdown_angle = self.compute_touchdown_angle()
            else:
                self.vy = 0.0
                # До апогея держим предыдущий угол (или базовый)
                self.touchdown_angle = BASE_HIP_ANGLE

            # Желаемый угол в полёте – угол приземления
            self.desired_hip_angle = self.touchdown_angle

            # Вызываем верхний уровень для коррекции уставок (на следующий цикл)
            self.high_level_control(s)

            if self.apex_reached and h < 0.03:
                self._set_phase(PRELAND)

        elif self.phase == PRELAND:
            # Продолжаем держать вычисленный угол приземления
            self.desired_hip_angle = self.touchdown_angle
            if touchdown:
                self._set_phase(LANDING)
                self.brake_start_time = None

        elif self.phase == LANDING:
            # Сразу после касания продолжаем держать touchdown_angle,
            # через 0.1 с переключаемся на прыжковый угол для следующего цикла
            if t - self.phase_start_time < 0.1:
                self.desired_hip_angle = self.touchdown_angle
            else:
                self.desired_hip_angle = self.jump_angle

            if (t - self.phase_start_time > LANDING_HOLD and abs(self.omega) < 5.0):
                self._set_phase(SPINUP)

    # ========== Регуляторы ==========
    def control_centrifugal(self):
        if self.phase == SPINUP:
            err = self.desired_spin_speed - self.omega
            tau = KP_SPIN * err
            tau = max(-GR_TORQUE_MAX, min(GR_TORQUE_MAX, tau))
            self.centrifugal_motor.setTorque(tau)
        elif self.phase == LIFTOFF:
            if self.brake_start_time is not None:
                if self.robot.getTime() - self.brake_start_time < BRAKE_DURATION:
                    self.centrifugal_motor.setTorque(-GR_TORQUE_MAX)
                else:
                    self.centrifugal_motor.setTorque(0.0)
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

    def control_hip(self):
        if self.phase == START:
            self.hip_motor.setTorque(0.0)
            return

        current = self.hip_sensor.getValue() if self.hip_sensor else 0.0
        err = self.desired_hip_angle - current
        der = err - self.prev_hip_error
        tau = KP_HIP * err + KD_HIP * der
        tau = max(-HIP_TORQUE_MAX, min(HIP_TORQUE_MAX, tau))
        self.hip_motor.setTorque(tau)
        self.prev_hip_error = err

    # ========== Служебные ==========
    def _set_phase(self, new_phase):
        self.phase = new_phase
        self.phase_start_time = self.robot.getTime()
        print(f"t={self.phase_start_time:.2f}s | -> {PHASE_NAMES[new_phase]}")

    def log_state(self, s):
        t = self.robot.getTime()
        if int(t * 1000) % 200 == 0:
            h = s['z'] - self.ground_z
            print(f"t={t:.2f}s | {PHASE_NAMES[self.phase]} | "
                  f"h={h:.3f}m | ω={self.omega:.1f} | φ_ref={self.desired_hip_angle:.3f} | "
                  f"φ={s['hip_angle']:.2f} | vx={self.vx:.2f}")

    def run(self):
        while self.robot.step(TIME_STEP) != -1:
            s = self.read_sensors()
            self.compute_omega(s)
            self.compute_velocity(s)
            self.update_phase(s)
            self.control_centrifugal()
            self.control_hip()
            self.log_state(s)

if __name__ == "__main__":
    SAHRController().run()