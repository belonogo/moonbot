#!/usr/bin/env python

# этот скрипт запускается также как и скрипт одометрии через python3 имя_файла.py

import rospy
# указываем тип сообщений топика
from std_msgs.msg import Int64

# указываем название топика (в нашем случае serv_info), тип данных который импортировали, длину очереди
pub = rospy.Publisher('bur_info', Int64, queue_size=10)
# чтобы топик работал необходимо создать ноду
rospy.init_node('bur_info_publisher')
r = rospy.Rate(10) # 10hz

while not rospy.is_shutdown():
    r.sleep()
