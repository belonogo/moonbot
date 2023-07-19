#!/usr/bin/env python

import rospy
# указываем тип сообщений топика
from std_msgs.msg import Int64

# указываем название топика (в нашем случае serv_info), тип данных который импортировали, длину очереди
pub = rospy.Publisher('serv_info', Int64, queue_size=10)
# чтобы топик работал необходимо создать ноду
rospy.init_node('serv_info_publisher')
r = rospy.Rate(10) # 10hz

while not rospy.is_shutdown():
    pub.publish("Hello World")
    r.sleep()
