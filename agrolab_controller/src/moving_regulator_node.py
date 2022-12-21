#!/usr/bin/env python3
# coding --utf-8--

from agrolab_controller.srv import custom_pose, custom_poseRequest, custom_poseResponse
from std_srvs.srv import Trigger, TriggerRequest, TriggerResponse
from sensor_msgs.msg import JointState
from geometry_msgs.msg import Point
from std_msgs.msg import Float64
import threading
import rospy
import time

class MovingRegulator:
    def __init__(self):
        rospy.init_node("moving_regulator_node")
        self.mode = "GAZEBO"                        # Режим работы RVIZ или GAZEBO

        self.publisher: rospy.Publisher = rospy.Publisher('/joint_states', JointState, queue_size=10)

        self.X_publisher = rospy.Publisher('/agrolab/base_link_to_X_controller/command', Float64, queue_size = 100)
        self.Y_publisher = rospy.Publisher('/agrolab/X_to_Y_controller/command', Float64, queue_size = 100)
        self.Z_publisher = rospy.Publisher('/agrolab/Y_to_Z_controller/command', Float64, queue_size = 100)

        self.X_subscriber = rospy.Subscriber('/agrolab/base_link_to_X_controller/command', Float64, self.x_callback)
        self.Y_subscriber = rospy.Subscriber('/agrolab/X_to_Y_controller/command', Float64, self.y_callback)
        self.Z_subscriber = rospy.Subscriber('/agrolab/Y_to_Z_controller/command', Float64, self.z_callback)

        self.detection_subscriber = rospy.Subscriber('/apple_detector/detected_object', Point, self.detection_callback)

        self.moving_in_process: bool = False      # Если флаг True, то идет поиск, если False - робот стоит на месте

        self.max_x: float = 0.43                    # Предельная координата по х
        self.max_y: float = 0.54                    # Предельная координата по у
        self.max_z: float = 0.2                     # Предельная координата по z
        self.x: float = 0                           # Текущая координата по х
        self.y: float = 0                           # Текущая координата по у
        self.z: float = 0.15                        # Текущая координата по z

        self.obj_x = 0
        self.obj_y = 0

        speed: int = 0.5                              # Скорость робота (коэффициент)
        self.x_path_delta = 0.03                    # Участок пути по х, который робот проходит за один прогон
        self.x_delta: float = 0.005                 # Cмещение по х за одну итерацию
        self.y_delta: float = 0.005                 # Cмещение по у за одну итерацию
        self.time_delta: float = 0.01/speed         # Промежуток времени на одну итерацию

        self.epsilon = 0.001

        # self.start_searching_service: rospy.Service = rospy.Service('~move', custom_pose, self.move_callback)     # Сервис ROS для возобновления "поиска"
        # self.stop_searching_service: rospy.Service = rospy.Service('~stop_moving', Trigger, self.stop_moving_callback)        # Сервис ROS для остановки "поиска"

        threading.Thread(target=self.displace_by_coords, daemon=True).start()    # Поток, в котором происходит управление перемещением робота

        self.start_moving_service: rospy.Service = rospy.Service('~start_moving', Trigger, self.start_moving_callback)     # Сервис ROS для возобновления "поиска"
        self.stop_moving_service: rospy.Service = rospy.Service('~stop_moving', Trigger, self.stop_moving_callback)        # Сервис ROS для остановки "поиска"

    def start_moving_callback(self, request: TriggerRequest):
        self.moving_in_process: bool = True
        return TriggerResponse()
    
    def stop_moving_callback(self, request: TriggerRequest):
        self.moving_in_process: bool = False
        return TriggerResponse()

    def x_callback(self, data: Float64):
        self.x = data.data
    
    def y_callback(self, data: Float64):
        self.y = data.data
    
    def z_callback(self, data: Float64):
        self.z = data.data
    
    def detection_callback(self, data: Point):
        self.obj_x = data.x - 1280/2
        self.obj_y = data.y - 720/2

    def displace_by_coords(self):
        while not rospy.is_shutdown():
            if self.moving_in_process and abs(self.obj_x) > 1280*self.epsilon and abs(self.obj_y) > 720*self.epsilon:
                if abs(self.obj_x) > 1280*self.epsilon:
                    if abs(self.obj_x) > 1280*self.epsilon*15:
                        self.y -= self.y_delta * self.obj_x/abs(self.obj_x)
                        rospy.loginfo(f"{self.x_delta=}")
                    else:
                        self.y -= self.y_delta/10 * self.obj_x/abs(self.obj_x)
                        rospy.loginfo(f"{self.x_delta/10=}")

                if abs(self.obj_y) > 720*self.epsilon:
                    if abs(self.obj_y) > 720*self.epsilon*20:
                        self.x -= self.x_delta * self.obj_y/abs(self.obj_y)
                        rospy.loginfo(f"{self.y_delta=}")
                    else:
                        self.x -= self.x_delta/10 * self.obj_y/abs(self.obj_y)
                        rospy.loginfo(f"{self.y_delta/10=}")

                self.X_publisher.publish(self.x)
                self.Y_publisher.publish(self.y)
                time.sleep(self.time_delta)
                rospy.logwarn(f"{self.obj_x}_{self.obj_y}")
                rospy.logerr(f"{self.x}_{self.y}")

if __name__ == "__main__":
    moving_reg = MovingRegulator()
    while not rospy.is_shutdown():
        pass