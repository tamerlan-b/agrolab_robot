#!/usr/bin/env python

import rospy
import smach
import smach_ros
from std_srvs.srv import Trigger, TriggerRequest, TriggerResponse
from geometry_msgs.msg import Point
from std_msgs.msg import Float64

state_upd_time = 0.5

class WaitState(smach.State):
    """Состояние ожидания запуска конечного автомата (КА)
    """

    def __init__(self):
        # Сервис для запуска КА
        self.start_fsm_service: rospy.Service = rospy.Service('~start_fsm', Trigger, self.start_callback)
        self.start_fsm = False

        # Сервис поиска объекта
        rospy.wait_for_service('/search_node/start_searching')
        self.start_search_client = rospy.ServiceProxy('/search_node/start_searching', Trigger)
        
        smach.State.__init__(self, outcomes=['start', 'remain'])
    
    def start_callback(self, request: TriggerRequest) -> TriggerResponse:
        self.start_fsm = True
        return TriggerResponse()

    def execute(self, userdata):
        global state_upd_time
        rospy.sleep(state_upd_time)
        if self.start_fsm:
            # Сбрасываем переменную
            self.start_fsm = False
            try:
                # Запускаем поиск объекта
                resp: TriggerResponse = self.start_search_client(TriggerRequest())
            except rospy.ServiceException as e:
                print("Service call failed: %s"%e)
            return 'start'
        else:
            return 'remain'

class SearchState(smach.State):
    """Состояние поиска объекта
    """

    def __init__(self):
        # Сервис для остановки КА
        self.stop_fsm_service: rospy.Service = rospy.Service('~stop_fsm', Trigger, self.stop_callback)
        self.stop_fsm = False
        self.see_object = False

        # Сервис остановки процедуры поиска
        rospy.wait_for_service('/search_node/stop_searching')
        self.stop_search_client = rospy.ServiceProxy('/search_node/stop_searching', Trigger)
        # Подписываемся на топик с координатами объекта
        rospy.Subscriber("/apple_detector/detected_object", Point, self.object_callback)
        smach.State.__init__(self, outcomes=['find_object', 'stop', 'remain'])

    def object_callback(self, msg: Point):
        self.see_object = True
        # rospy.loginfo("See object")
    
    def stop_callback(self, request: TriggerRequest) -> TriggerResponse:
        self.stop_fsm = True
        return TriggerResponse()

    def stop_search(self):
        try:
            # Останавливаем поиск объекта
            resp: TriggerResponse = self.stop_search_client(TriggerRequest())
        except rospy.ServiceException as e:
            print("Service call failed: %s"%e)

    def execute(self, userdata):
        global state_upd_time
        rospy.sleep(state_upd_time)
        
        if self.stop_fsm:
            self.stop_fsm = False
            self.stop_search()
            return 'stop'
        
        if self.see_object:
            self.see_object = False
            self.stop_search()
            return 'find_object'
        
        return 'remain'

class GoToObjectState(smach.State):
    """Состояние приближения к объекту и занятия положения над ним
    """

    def __init__(self):
        rospy.wait_for_service('/moving_regulator_node/start_moving')
        self.displace_publisher = rospy.Publisher('/moving_regulator_node/displace', Point)

        self.move_to_object_client = rospy.ServiceProxy('/moving_regulator_node/start_moving', Trigger)
        self.stop_move_to_object_client = rospy.ServiceProxy('/moving_regulator_node/stop_moving', Trigger)

        self.detection_client = rospy.Subscriber('/apple_detector/detected_object', Point, self.detection_callback)
        self.is_moving = False
        smach.State.__init__(self, outcomes=['arrive', 'remain'])

        self.detected_obj_x = 1000
        self.detected_obj_y = 1000
        self.staying_counter = 0
        self.max_staying_count = 10
    
    def detection_callback(self, data: Point):
        self.detected_obj_x = data.x - 1280/2
        self.detected_obj_y = data.y - 720/2

    def move_to_object(self):
        try:
            # Останавливаем поиск объекта
            resp: TriggerResponse = self.move_to_object_client(TriggerRequest())
        except rospy.ServiceException as e:
            print("Service call failed: %s"%e)

    def stop_moving_to_object(self):
        try:
            # Останавливаем поиск объекта
            resp: TriggerResponse = self.stop_move_to_object_client(TriggerRequest())
        except rospy.ServiceException as e:
            print("Service call failed: %s"%e)

    def execute(self, userdata):
        global state_upd_time
        rospy.sleep(state_upd_time)

        while True:
            if not self.is_moving:
                self.move_to_object()
                self.is_moving = True
            
            if abs(self.detected_obj_x) <= 10 and abs(self.detected_obj_y) <= 10:
                self.staying_counter += 1
                rospy.sleep(0.1)
            else:
                self.staying_counter = 0
            
            if self.staying_counter > self.max_staying_count:
                # self.X_publisher.publ1ish(-0.043)
                self.stop_moving_to_object()
                rospy.sleep(2)
                msg = Point()
                msg.x = -0.04
                msg.y = 0.0
                msg.z = 0.0
                self.displace_publisher.publish(msg)
                rospy.sleep(1)
                return 'arrive' 

        return 'remain'

class GrabState(smach.State):
    """Состояние захвата объекта
    """

    def __init__(self):
        smach.State.__init__(self, outcomes=['grabbed', 'remain'])

        self.Z_publisher = rospy.Publisher('/agrolab/Y_to_Z_controller/command', Float64, queue_size = 100)
        self.gripper_1_publisher = rospy.Publisher('/agrolab/gripper_controller/command', Float64, queue_size = 100)
        self.gripper_2_publisher = rospy.Publisher('/agrolab/gripper_sub_controller/command', Float64, queue_size = 100)
        self.absolute_moving = rospy.Publisher('/moving_regulator_node/absolute_moving', Point, queue_size = 100)

    def execute(self, userdata):
        global state_upd_time
        rospy.sleep(state_upd_time)
        self.gripper_1_publisher.publish(0.1)
        self.gripper_2_publisher.publish(0.1)
        rospy.sleep(0.5)
        # msg = Point()
        # msg.x = -1
        # msg.y = -1
        # msg.z = 0.2
        # self.absolute_moving.publish(msg)
        self.Z_publisher.publish(0.2)
        rospy.sleep(1)
        self.gripper_1_publisher.publish(0.0)
        self.gripper_2_publisher.publish(0.0)
        rospy.sleep(0.5)
        # msg = Point()
        # msg.x = -1
        # msg.y = -1
        # msg.z = 0.0
        # self.absolute_moving.publish(msg)
        self.Z_publisher.publish(0.0)
        rospy.sleep(1)
        # return 'remain'
        return 'grabbed'

class MoveObjectState(smach.State):
    """Состояние перемещения объекта в целевую точку
    """
    
    def __init__(self):
        smach.State.__init__(self, outcomes=['arrive', 'remain'])
        self.Z_publisher = rospy.Publisher('/agrolab/Y_to_Z_controller/command', Float64, queue_size = 100)
        self.gripper_1_publisher = rospy.Publisher('/agrolab/gripper_controller/command', Float64, queue_size = 100)
        self.gripper_2_publisher = rospy.Publisher('/agrolab/gripper_sub_controller/command', Float64, queue_size = 100)
        self.absolute_moving = rospy.Publisher('/moving_regulator_node/absolute_moving', Point, queue_size = 100)

    def execute(self, userdata):
        global state_upd_time
        rospy.sleep(state_upd_time)

        msg = Point()
        msg.x = 0.08
        msg.y = 0.3175
        msg.z = -1
        self.absolute_moving.publish(msg)
        rospy.sleep(3)

        self.Z_publisher.publish(0.08)
        rospy.sleep(1)

        self.gripper_1_publisher.publish(0.1)
        self.gripper_2_publisher.publish(0.1)
        rospy.sleep(0.5)

        self.Z_publisher.publish(0.0)
        rospy.sleep(1)

        self.gripper_1_publisher.publish(0.0)
        self.gripper_2_publisher.publish(0.0)
        rospy.sleep(1)
        
        # return 'remain'
        return 'arrive'

class ReleaseState(smach.State):
    """Состояние установки объекта в целевую точку
    """

    def __init__(self):
        smach.State.__init__(self, outcomes=['released', 'remain'])

    def execute(self, userdata):
        global state_upd_time
        rospy.sleep(state_upd_time)
        # return 'remain'
        return 'released'

class ReturnHomeState(smach.State):
    """Состояние возвращения в домашнюю точку
    """

    def __init__(self):
        smach.State.__init__(self, outcomes=['homed', 'remain'])
        self.absolute_moving = rospy.Publisher('/moving_regulator_node/absolute_moving', Point, queue_size = 100)

    def execute(self, userdata):
        global state_upd_time
        rospy.sleep(state_upd_time)

        msg = Point()
        msg.x = 0.08
        msg.y = 0.3175
        msg.z = -1
        self.absolute_moving.publish(msg)
        rospy.sleep(3)

        return 'remain'
        return 'homed'


# main
def main():
    rospy.init_node('state_machine_node')

    # Создаем машину состояний
    sm = smach.StateMachine(outcomes=['finished'])

    # Добавляем состояния
    with sm:

        smach.StateMachine.add('Wait', WaitState(), transitions={'start':'Search',
                                                                    'remain': 'Wait'})

        smach.StateMachine.add('Search', SearchState(), transitions={'find_object':'GoToObject', 
                                                                        'stop':'Wait',
                                                                        'remain': 'Search'})

        smach.StateMachine.add('GoToObject', GoToObjectState(), transitions={'arrive':'Grab',
                                                                                'remain': 'GoToObject'})

        smach.StateMachine.add('Grab', GrabState(), transitions={'grabbed':'MoveObject',
                                                                    'remain': 'Grab'})

        smach.StateMachine.add('MoveObject', MoveObjectState(), transitions={'arrive':'Release',
                                                                                'remain': 'MoveObject'})
                                                                            
        smach.StateMachine.add('Release', ReleaseState(), transitions={'released':'ReturnHome',
                                                                        'remain': 'Release'})

        smach.StateMachine.add('ReturnHome', ReturnHomeState(), transitions={'homed':'Search',
                                                                                'remain': 'ReturnHome'})

    # Визуализируем состояния
    sis = smach_ros.IntrospectionServer('agrolab', sm, '/SM_ROOT')
    sis.start()

    # Запускаем машину состояний
    outcome = sm.execute()

    rospy.spin()
    sis.stop()


if __name__ == '__main__':
    main()