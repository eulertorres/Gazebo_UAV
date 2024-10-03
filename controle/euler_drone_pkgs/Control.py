#!/usr/bin/env python

import rospy
from sensor_msgs.msg import Joy
from mavros_msgs.msg import OverrideRCIn

def joy_callback(data):
    # Crie a mensagem RC Override
    rc_override = OverrideRCIn()

    # Mapear eixos do joystick para canais RC
    # Cada canal RC vai de 1000 a 2000
    # Supondo que o eixo 1 (left/right) controla roll e eixo 2 (up/down) controla pitch
    rc_override.channels[0] = int(1500 + data.axes[0] * 500)  # Roll (canal 1)
    rc_override.channels[1] = int(1500 + data.axes[1] * 500)  # Pitch (canal 2)
    rc_override.channels[2] = int(1500 + data.axes[2] * 500)  # Throttle (canal 3)
    rc_override.channels[3] = int(1500 + data.axes[3] * 500)  # Yaw (canal 4)
    rc_override.channels[4] = int(1500 + data.axes[4] * 500)  # Yaw (canal 4)

    # Publique a mensagem RC Override no tópico apropriado
    rc_pub.publish(rc_override)

rospy.init_node('joy_to_rc_override')
rc_pub = rospy.Publisher('/mavros/rc/override', OverrideRCIn, queue_size=10)
rospy.Subscriber("/joy", Joy, joy_callback)

rospy.spin()
