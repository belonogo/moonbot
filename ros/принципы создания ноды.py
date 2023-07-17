#!/usr/bin/env python

# импорт библиотек для работы с ros
import rospy
#import roslib
#roslib.load_manifest('differential_drive')
import tf
from tf.broadcaster import TransformBroadcaster

# импорт библитек для работы с разными типами сообщений в топиках
from geometry_msgs.msg import Quaternion
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Vector3
from nav_msgs.msg import Odometry
from std_msgs.msg import Int16, Int32, Int64, UInt32

# импорт библитек для расчета одометрии
from math import sin, cos, pi

class DiffTf: # название класса ноды
    def __init__(self): # конструктор
        rospy.init_node("diff_tf") # название ноды, которое будет отображаться в rosnode list
        self.nodename = rospy.get_name()
        rospy.loginfo("-I- %s started" % self.nodename)
        
        #### parameters #######

        #Wheel radius : 0.032
        # wheel circum = 2* 3.14 * 0.0325 = 0.2010
        # One rotation encoder ticks : 9 ticks
        # For 1 meter: 9 * ( 1 / 0.2010) = 48 ticks

        # поля класса (глобальные переменные внутри класса) в которых будут результаты расчета одометрии

        # все поля, данные в которые поступают через rospy.get_param придется указывать вручную, тк нет urdf описания робота

        self.rate = rospy.get_param('~rate',10.0)  # диапазон публикации в топик
        self.ticks_meter = float(rospy.get_param('ticks_meter', 190))  # количество тиков на метр
        self.base_width = float(rospy.get_param('~base_width', 0.11)) # ширина робота
        
        self.base_frame_id = rospy.get_param('~base_frame_id','base_link') # base_frame робота
        self.odom_frame_id = rospy.get_param('~odom_frame_id', 'odom') # одометрия рообота
        
        self.encoder_min = rospy.get_param('encoder_min', -2147483648) # минимальное количество тиков
        self.encoder_max = rospy.get_param('encoder_max', 2147483648) # максимальное количество тиков
        self.encoder_low_wrap = rospy.get_param('wheel_low_wrap', (self.encoder_max - self.encoder_min) * 0.3 + self.encoder_min )
        self.encoder_high_wrap = rospy.get_param('wheel_high_wrap', (self.encoder_max - self.encoder_min) * 0.7 + self.encoder_min )
 
        self.t_delta = rospy.Duration(1.0/self.rate)
        self.t_next = rospy.Time.now() + self.t_delta
        
        # internal data
        self.enc_left = None        # wheel encoder readings
        self.enc_right = None
        self.left = 0               # actual values coming back from robot
        self.right = 0
        self.lmult = 0
        self.rmult = 0
        self.prev_lencoder = 0
        self.prev_rencoder = 0
        self.x = 0                  # position in xy plane 
        self.y = 0
        self.th = 0
        self.dx = 0                 # speeds in x/rotation
        self.dr = 0

        self.yaw = 0.01
        self.pitch = 0.01
        self.roll = 0.01

        self.then = rospy.Time.now()

        self.quaternion_1 = Quaternion()
        
        # создание подписчиков на паблишеры с ардуино
        rospy.Subscriber("wheel_ticks_left", UInt32, self.lwheelCallback) # 1-ый аргумент - название топика, 2-ой - тип данных сообщения в топике, 3-ий метод класса, обрабатывающий сообщение
        rospy.Subscriber("wheel_ticks_right", UInt32, self.rwheelCallback)
        rospy.Subscriber("imu_data", Vector3, self.imu_value_update)

        #создание паблишера на топик одометрии
        self.odomPub = rospy.Publisher("odom", Odometry, queue_size=10)
        self.odomBroadcaster = TransformBroadcaster()

    # основной цикл ноды, аналог loop в arduino    
    def spin(self):
        r = rospy.Rate(self.rate)
        while not rospy.is_shutdown():
            self.update()
            r.sleep()
       
    # обновление значений
    def update(self):
        now = rospy.Time.now()
        if now > self.t_next:
            elapsed = now - self.then
            self.then = now
            elapsed = elapsed.to_sec()
            

            # calculate odometry
            if self.enc_left == None:
                d_left = 0
                d_right = 0
            else:
                d_left = (self.left - self.enc_left) / self.ticks_meter
                d_right = (self.right - self.enc_right) / self.ticks_meter
            self.enc_left = self.left
            self.enc_right = self.right
           
            # distance traveled is the average of the two wheels 
            d = ( d_left + d_right ) / 2
            # this approximation works (in radians) for small angles
            th = ( d_right - d_left ) / self.base_width
            # calculate velocities
            self.dx = d / elapsed
            self.dr = th / elapsed
           

             
            if (d != 0):
                # calculate distance traveled in x and y
                x = cos( th ) * d
                y = -sin( th ) * d
                # calculate the final position of the robot
                self.x = self.x + ( cos( self.th ) * x - sin( self.th ) * y )
                self.y = self.y + ( sin( self.th ) * x + cos( self.th ) * y )
            if( th != 0):
                self.th = self.th + th
                
            # publish the odom information
            quaternion = Quaternion()


            quaternion.x = 0.0
            quaternion.y = 0.0

            quaternion.z = sin( self.th / 2 )
            quaternion.w = cos( self.th / 2 )


            # публикация одометрии в паблишере
            self.odomBroadcaster.sendTransform(
                (self.x, self.y, 0),
                (quaternion.x, quaternion.y, quaternion.z, quaternion.w),
                rospy.Time.now(),
                self.base_frame_id,
                self.odom_frame_id
                )
            
            odom = Odometry()
            odom.header.stamp = now
            odom.header.frame_id = self.odom_frame_id
            odom.pose.pose.position.x = self.x
            odom.pose.pose.position.y = self.y
            odom.pose.pose.position.z = 0
            odom.pose.pose.orientation = quaternion
            odom.child_frame_id = self.base_frame_id
            odom.twist.twist.linear.x = self.dx
            odom.twist.twist.linear.y = 0
            odom.twist.twist.angular.z = self.dr
            self.odomPub.publish(odom)

    # метод класса, обрабатывающий imu данные
    def imu_value_update(self, imu_data):
        orient = imu_data

        self.yaw = orient.x
        self.pitch = orient.y
        self.roll = orient.z

        try:	
            self.quaternion_1 = tf.transformations.quaternion_from_euler(self.yaw, self.pitch, self.roll)

        except:
            rospy.logwarn("Unable to get quaternion values")
            pass

    # метод класса, обрабатывающий левые тики
    def lwheelCallback(self, msg):
        enc = msg.data
        if (enc < self.encoder_low_wrap and self.prev_lencoder > self.encoder_high_wrap):
            self.lmult = self.lmult + 1
            
        if (enc > self.encoder_high_wrap and self.prev_lencoder < self.encoder_low_wrap):
            self.lmult = self.lmult - 1
            
        self.left = 1.0 * (enc + self.lmult * (self.encoder_max - self.encoder_min)) 


        self.prev_lencoder = enc

    # метод класса, обрабатывающий правые тики   
    def rwheelCallback(self, msg):
        enc = msg.data
        if(enc < self.encoder_low_wrap and self.prev_rencoder > self.encoder_high_wrap):
            self.rmult = self.rmult + 1
        
        if(enc > self.encoder_high_wrap and self.prev_rencoder < self.encoder_low_wrap):
            self.rmult = self.rmult - 1
            
        self.right = 1.0 * (enc + self.rmult * (self.encoder_max - self.encoder_min))


        self.prev_rencoder = enc

# запуск ноды, аналог main
if __name__ == '__main__':
    """ main """
    diffTf = DiffTf()
    diffTf.spin()
