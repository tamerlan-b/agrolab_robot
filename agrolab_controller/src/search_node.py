#!/usr/bin/env python3
# coding --utf-8--

from std_srvs.srv import Trigger, TriggerRequest, TriggerResponse
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64
import threading
import rospy
import time

class Searcher:
    def __init__(self):
        rospy.init_node("search_node")
        self.mode = "GAZEBO"                        # Режим работы RVIZ или GAZEBO

        self.publisher: rospy.Publisher = rospy.Publisher('/joint_states', JointState, queue_size=10)

        self.X_publisher = rospy.Publisher('/agrolab/base_link_to_X_controller/command', Float64, queue_size = 100)
        self.Y_publisher = rospy.Publisher('/agrolab/X_to_Y_controller/command', Float64, queue_size = 100)
        self.Z_publisher = rospy.Publisher('/agrolab/Y_to_Z_controller/command', Float64, queue_size = 100)

        self.max_x: float = 0.43                    # Предельная координата по х
        self.max_y: float = 0.54                    # Предельная координата по у
        self.max_z: float = 0.2                     # Предельная координата по z
        self.x: float = 0                           # Текущая координата по х
        self.y: float = 0                           # Текущая координата по у
        self.z: float = 0.15                        # Текущая координата по z

        self.searching_in_process: bool = True      # Если флаг True, то идет поиск, если False - робот стоит на месте

        speed: int = 2                              # Скорость робота (коэффициент)
        self.x_path_delta = 0.03                    # Участок пути по х, который робот проходит за один прогон
        self.x_delta: float = 0.005                 # Cмещение по х за одну итерацию
        self.y_delta: float = 0.005                 # Cмещение по у за одну итерацию
        self.time_delta: float = 0.01/speed         # Промежуток времени на одну итерацию

        self.moving_forward: bool = True            # Если флаг True, то движение по y - вперед, если False, то назад
        self.moving_right: bool = True              # Если флаг True, то движение по x - вправо, если False, то влево
        self.current_moving: str = "y"              # Сейчас происходит движение по x или по y

        self.start_searching_service: rospy.Service = rospy.Service('~start_searching', Trigger, self.start_searching_callback)     # Сервис ROS для возобновления "поиска"
        self.stop_searching_service: rospy.Service = rospy.Service('~stop_searching', Trigger, self.stop_searching_callback)        # Сервис ROS для остановки "поиска"

        threading.Thread(target=self.searching, daemon=True).start()    # Поток, в котором происходит управление перемещением робота

    def start_searching_callback(self, request: TriggerRequest) -> TriggerResponse:
        """Callback для сервиса возобновления "поиска"

        Args:
            request (TriggerRequest): Запрос (путой, т.к. тип TriggerRequest)

        Returns:
            TriggerResponse: Ответ на запрос (пустой, т.к. тип TriggerResponse)
        """
        self.searching_in_process = True
        return TriggerResponse()
    
    def stop_searching_callback(self, request: TriggerRequest) -> TriggerResponse:
        """Callback для сервиса остановки "поиска"

        Args:
            request (TriggerRequest): Запрос (путой, т.к. тип TriggerRequest)

        Returns:
            TriggerResponse: Ответ на запрос (пустой, т.к. тип TriggerResponse)
        """
        self.searching_in_process = False
        return TriggerResponse()

    def searching(self) -> None:
        """Функция для управления перемещением робота при выполнении "поиска"
        """
        current_x_path: float = 0   # Счетчик для суммарных смещений по х в рамках одного прогона
        while not rospy.is_shutdown():
            # Выполняется только если "поиск" не остановлен
            if self.searching_in_process:

                # обработка перемещений по у
                if self.current_moving == "y":      
                    if self.moving_right and self.y + self.y_delta < self.max_y:
                        self.y += self.y_delta
                    elif self.moving_right and self.y + self.y_delta >= self.max_y:
                        self.y = self.max_y
                        self.moving_right = False
                        self.current_moving = "x"
                    elif not self.moving_right and self.y - self.y_delta > 0:
                        self.y -= self.y_delta
                    elif not self.moving_right and self.y - self.y_delta <= 0:
                        self.y = 0
                        self.moving_right = True
                        self.current_moving = "x"
                
                # обработка перемещений по х
                elif self.current_moving == "x":    
                    if self.moving_forward and self.x + self.x_delta < self.max_x:
                        self.x += self.x_delta
                    elif self.moving_forward and self.x + self.x_delta >= self.max_x:
                        self.x = self.max_x
                        self.moving_forward = False
                    elif not self.moving_forward and (self.x == self.max_x or (self.x < self.max_x and self.x - self.x_delta > 0)):
                        self.x -= self.x_delta
                    elif not self.moving_forward and self.x - self.x_delta < 0:
                        self.x = 0
                        self.moving_forward = True
                    
                    if abs(current_x_path - self.x_path_delta) < 10**(-5):
                        self.current_moving = "y"
                        current_x_path = 0
                    else:
                        current_x_path += self.x_delta

                # В зависимости от режима работы происходит управление либо моделью в Gazebo, либо в Rviz
                if self.mode == "GAZEBO":
                    self.X_publisher.publish(self.x)
                    self.Y_publisher.publish(self.y)
                    self.Z_publisher.publish(self.z)
                elif self.mode == "RVIZ":
                    msg: JointState = self.get_msg(self.x, self.y, self.z)  # Генерация сообщения с обновленными координатами
                    self.publisher.publish(msg)                             # Отправка сообщения в топик для перемещения робота

                time.sleep(self.time_delta)                             # Временная задержка перед следующей итерацией

    def get_msg(self, x: float, y: float, z: float) -> JointState:
        """Функция для генерации сообщения для отправки в топик управления роботом

        Args:
            x (float): Координата х
            y (float): Координата у
            z (float): Координата z

        Returns:
            JointState: Сгенерированное сообщение
        """
        msg: JointState = JointState()
        msg.header.stamp.secs = rospy.get_rostime().secs
        msg.header.stamp.nsecs = rospy.get_rostime().nsecs
        msg.name = ["base_link_to_X", "X_to_Y", "Y_to_Z"]
        msg.position = [x, y, z]
        return msg

if __name__ == "__main__":
    searcher = Searcher()
    while not rospy.is_shutdown():
        pass