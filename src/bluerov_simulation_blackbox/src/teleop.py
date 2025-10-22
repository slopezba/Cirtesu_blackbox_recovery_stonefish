#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Joy
from std_msgs.msg import Bool
from std_msgs.msg import Int32


class JoyControl:
    def __init__(self):
        self.axes = []
        self.buttons = []
        self.velocidad = 0.5  # Velocidad base

        self.initNode()

    def initNode(self):
        rospy.init_node('simula_envio', anonymous=True)
        self.pub = rospy.Publisher("/cmd_vel", Twist, queue_size=10)
        self.arm_pub = rospy.Publisher("/arm_disarm", Bool, queue_size=10)
        self.luces_pub = rospy.Publisher("/luces_pwm", Int32, queue_size=10)
        rospy.Subscriber('/joy', Joy, self.joy_callback)
        rospy.Timer(rospy.Duration(0.1), self.send_cmd_vel)

        print("Nodo iniciado")
        rospy.spin()

    def joy_callback(self, data):
        self.axes = data.axes
        self.buttons = data.buttons
        # print(f"Ejes: {self.axes}\nBotones: {self.buttons}")
        # armar
        if len(self.buttons) > 0 and self.buttons[7] == 1:
            self.arm_pub.publish(Bool(data=True))
            rospy.loginfo("ARMAR")

        # Botón B (índice 1) para desarmar
        if len(self.buttons) > 1 and self.buttons[6] == 1:
            self.arm_pub.publish(Bool(data=False))
            rospy.loginfo("DESARMAR")

        # Encender luces (PWM alto)
        if len(self.buttons) > 2 and self.axes[6] == -1:
            self.luces_pub.publish(Int32(data=1900))
            rospy.loginfo("Luces encendidas")

        # Apagar luces (PWM bajo)
        if len(self.buttons) > 3 and self.axes[6] == 1:
            self.luces_pub.publish(Int32(data=0))
            rospy.loginfo("Luces apagadas")

    def send_cmd_vel(self, event):
        if not self.axes:
            return  # No hay datos aún

        mov = Twist()
        mov.linear.x  = self.velocidad * self.axes[1]   # Adelante/Atrás
        mov.linear.y  = - self.velocidad * self.axes[0]   # Izq/Derecha
        mov.linear.z  = self.velocidad * self.axes[4]   # Subir/Bajar
        mov.angular.z = - self.velocidad * self.axes[3]   # Rotar


        self.pub.publish(mov)

if __name__ == "__main__":
    try:
        JoyControl()
    except rospy.ROSInterruptException:
        pass
