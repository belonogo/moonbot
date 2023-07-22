#!/usr/bin/env python

# импорт библиотек для работы с ros
import rospy

# импорт библитек для работы с разными типами сообщений в топиках
from std_msgs.msg import Int64

class DiffBur: # название класса ноды
    def __init__(self): # конструктор
        rospy.init_node("bur_info_publisher") # название ноды, которое будет отображаться в rosnode list
        self.nodename = rospy.get_name()
        rospy.loginfo("-I- %s started" % self.nodename)
        
        # поля класса (глобальные переменные внутри класса)
        
        #создание паблишера на топик bur_info
        self.burPub = rospy.Publisher("bur_info", Int64, queue_size=10)

    # основной цикл ноды, аналог loop в arduino    
    def spin(self):
        r = rospy.Rate(self.rate)
        while not rospy.is_shutdown():
            self.update()
            r.sleep()

# запуск ноды, аналог main
if __name__ == '__main__':
    """ main """
    diffBur = DiffBur()
    diffBur.spin()
