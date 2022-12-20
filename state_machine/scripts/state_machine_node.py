#!/usr/bin/env python

import rospy
import smach
import smach_ros
from std_srvs.srv import Trigger, TriggerRequest, TriggerResponse


class WaitState(smach.State):
    """Состояние ожидания запуска конечного автомата (КА)
    """

    def __init__(self):
        # Сервис для запуска КА
        self.start_fsm_service: rospy.Service = rospy.Service('~start_fsm', Trigger, self.start_callback)
        self.start_fsm = False
        smach.State.__init__(self, outcomes=['start', 'wait'])
    
    def start_callback(self, request: TriggerRequest) -> TriggerResponse:
        self.start_fsm = True
        return TriggerResponse()

    def execute(self, userdata):
        rospy.sleep(2.0)
        return 'start'  if self.start_fsm else 'wait'

class SearchState(smach.State):
    """Состояние поиска объекта
    """

    def __init__(self):
        smach.State.__init__(self, outcomes=['find_object', 'stop', 'searching'])

    def execute(self, userdata):
        rospy.sleep(2.0)
        return 'searching'
        return 'find_object'

class GrabState(smach.State):
    """Состояние захвата объекта
    """

    def __init__(self):
        smach.State.__init__(self, outcomes=['grabbed'])

    def execute(self, userdata):
        rospy.sleep(2.0)
        return 'grabbed'

class MoveState(smach.State):
    """Состояние перемещения объекта и его установки
    """
    
    def __init__(self):
        smach.State.__init__(self, outcomes=['released'])

    def execute(self, userdata):
        rospy.sleep(2.0)
        return 'released'

class ReturnHomeState(smach.State):
    """Состояние возвращения в домашнюю точку
    """

    def __init__(self):
        smach.State.__init__(self, outcomes=['homed'])

    def execute(self, userdata):
        rospy.sleep(2.0)
        return 'homed'


# main
def main():
    rospy.init_node('state_machine_node')

    # Создаем машину состояний
    sm = smach.StateMachine(outcomes=['finished'])

    # Добавляем состояния
    with sm:

        smach.StateMachine.add('Wait', WaitState(), transitions={'start':'Search',
                                                                'wait': 'Wait'})
        smach.StateMachine.add('Search', SearchState(), transitions={
                                                                    'find_object':'Grab', 
                                                                    'stop':'Wait',
                                                                    'searching': 'Search'})
        smach.StateMachine.add('Grab', GrabState(), transitions={'grabbed':'Move'})
        smach.StateMachine.add('Move', MoveState(), transitions={'released':'ReturnHome'})
        smach.StateMachine.add('ReturnHome', ReturnHomeState(), transitions={'homed':'Search'})

    # Визуализируем состояния
    sis = smach_ros.IntrospectionServer('agrolab', sm, '/SM_ROOT')
    sis.start()

    # Запускаем машину состояний
    outcome = sm.execute()

    rospy.spin()
    sis.stop()


if __name__ == '__main__':
    main()