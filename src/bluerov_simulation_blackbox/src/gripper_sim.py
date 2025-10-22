#!/usr/bin/env python3
import rospy
from std_msgs.msg import Int32
from sensor_msgs.msg import JointState
import math

# --- Límites de posición del gripper (rad) ---
RAD_OPEN  = -0.40   # posición "abrir"
RAD_CLOSE = -0.05   # posición "cerrar"

# --- Límites de velocidad y control ---
V_LIMIT = 0.20      # rad/s  (más bajo = más lento y suave)
A_LIMIT = None      # rad/s^2  (pon un número, p.ej. 1.0, para limitar aceleración)
CTRL_HZ = 100       # Hz

# --- PWM esperados ---
OPEN_PWM  = 1900
CLOSE_PWM = 1100

# --- Joints ---
JOINT_NAMES = ["bluerov2/gripper/joint1", "bluerov2/gripper/joint2"]
OPPOSITE_JOINTS = False  # pon True si deben ir opuestas [pos, -pos]


def clamp(x, xmin, xmax):
    return max(xmin, min(x, xmax))


class GripperController:
    def __init__(self):
        rospy.init_node("gripper_controller")

        # Estados
        self.current_pos = RAD_CLOSE  # arranca cerrado
        self.target_pos  = self.current_pos
        self.current_vel = 0.0

        # Pub/Sub
        self.pub = rospy.Publisher("/bluerov2/servo_commands", JointState, queue_size=10)
        rospy.Subscriber("/gripper_pwm", Int32, self.pwm_cb)

        # Timer de control
        self.dt = 1.0 / float(CTRL_HZ)
        rospy.Timer(rospy.Duration(self.dt), self.control_step)

    # Mapea PWM -> posición objetivo (lineal entre cerrar y abrir)
    def pwm_to_target(self, pwm: int) -> float:
        pwm_clamped = clamp(pwm, CLOSE_PWM, OPEN_PWM)
        # alpha en [0,1]: 0=cerrar, 1=abrir
        alpha = float(pwm_clamped - CLOSE_PWM) / float(OPEN_PWM - CLOSE_PWM)
        return RAD_CLOSE + alpha * (RAD_OPEN - RAD_CLOSE)

    def pwm_cb(self, msg: Int32):
        pwm = msg.data

        self.target_pos = self.pwm_to_target(pwm)

    def control_step(self, _):
        # Control tipo "perfil trapezoidal" sencillo:
        # 1) Calcula error de posición
        pos_err = self.target_pos - self.current_pos

        # 2) Calcula velocidad deseada limitada por V_LIMIT y acercándose al objetivo
        if abs(pos_err) < 1e-4:
            desired_vel = 0.0
        else:
            # velocidad hacia el objetivo con límite
            desired_vel = clamp(pos_err / self.dt, -V_LIMIT, V_LIMIT)

        if A_LIMIT is not None:
            # Limitador de aceleración (suaviza cambios de velocidad)
            max_dv = A_LIMIT * self.dt
            dv = clamp(desired_vel - self.current_vel, -max_dv, max_dv)
            self.current_vel += dv
        else:
            # Sin limitador de aceleración
            self.current_vel = desired_vel

        # 3) Integra posición con la velocidad resultante
        self.current_pos += self.current_vel * self.dt

        # 4) Asegura no salir de [RAD_OPEN, RAD_CLOSE]
        lo = min(RAD_OPEN, RAD_CLOSE)
        hi = max(RAD_OPEN, RAD_CLOSE)
        self.current_pos = clamp(self.current_pos, lo, hi)

        # 5) Publica estado actual como JointState
        js = JointState()
        js.header.stamp = rospy.Time.now()
        js.name = JOINT_NAMES
        if OPPOSITE_JOINTS:
            js.position = [self.current_pos, -self.current_pos]
        else:
            js.position = [self.current_pos, self.current_pos]
        self.pub.publish(js)


if __name__ == "__main__":
    try:
        GripperController()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
