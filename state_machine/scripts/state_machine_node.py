#!/usr/bin/env python

import rospy
import smach
import smach_ros


class WaitState(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['start'])

    def execute(self, userdata):
        rospy.sleep(2.0)
        return 'start'

class SearchState(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['find_object', 'stop'])

    def execute(self, userdata):
        rospy.sleep(2.0)
        return 'find_object'

class GrabState(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['grabbed'])

    def execute(self, userdata):
        rospy.sleep(2.0)
        return 'grabbed'

class MoveState(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['released'])

    def execute(self, userdata):
        rospy.sleep(2.0)
        return 'released'

class ReturnHomeState(smach.State):
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

        smach.StateMachine.add('Wait', WaitState(), transitions={'start':'Search'})
        smach.StateMachine.add('Search', SearchState(), transitions={
                                                                    'find_object':'Grab', 
                                                                    'stop':'Wait'})
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