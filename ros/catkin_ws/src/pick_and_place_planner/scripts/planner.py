#!/usr/bin/env python

import rospy
import smach
import smach_ros
from random import random

from pick_and_place_planner.msg import \
    PickAction, PickGoal, PickResult, PickFeedback, \
    PlaceAction, PlaceGoal, PlaceResult, PlaceFeedback


# 'SELECT' state
class Select(smach.State):

    def __init__(self):
        smach.State.__init__(self, outcomes=['selected', 'finished'])
        self.remaining_blocks = 3

    def execute(self, userdata):
        rospy.loginfo('Executing state: Select')
        if self.remaining_blocks > 0:
            self.remaining_blocks -= 1
            return 'selected'
        else:
            return 'finished'


# 'PICK' state
class Pick(smach.State):

    def __init__(self):
        smach.State.__init__(self, outcomes=['picked', 'failed'])

    def execute(self, userdata):
        rospy.loginfo('Executing state: Pick')
        if random() < 0.9:
            return 'picked'
        else:
            return 'failed'


# 'PLACE' state
class Place(smach.State):

    def __init__(self):
        smach.State.__init__(self, outcomes=['placed', 'failed'])

    def execute(self, userdata):
        rospy.loginfo('Executing state: Place')
        if random() < 0.9:
            return 'placed'
        else:
            return 'failed'


def main():
    rospy.init_node('pick_and_place_state_machine')

    # Create a SMACH state machine
    sm = smach.StateMachine(outcomes=['finished', 'failed'])

    # Open the container
    with sm:
        # Add states
        smach.StateMachine.add('SELECT', Select(),
                               transitions={'selected': 'PICKANDPLACE',
                                            'finished': 'finished'})

        # Submachine for pick-and-place
        sm_pickplace = smach.StateMachine(outcomes=['placed', 'failed'])
        with sm_pickplace:
            smach.StateMachine.add('PICK',
                                   smach_ros.SimpleActionState(
                                       'pick_action_server',
                                       PickAction),
                                   transitions={'succeeded': 'PLACE',
                                                'preempted': 'failed',
                                                'aborted': 'failed'})
            smach.StateMachine.add('PLACE',
                                   smach_ros.SimpleActionState(
                                       'place_action_server',
                                       PlaceAction),
                                   transitions={'succeeded': 'placed',
                                                'preempted': 'failed',
                                                'aborted': 'failed'})
        smach.StateMachine.add('PICKANDPLACE', sm_pickplace,
                               transitions={'placed': 'SELECT',
                                            'failed': 'failed'})

    # Create and start the introspection server
    sis = smach_ros.IntrospectionServer('sm_introspection_server', sm, '/SM_ROOT')
    sis.start()

    # Execute SMACH plan
    outcome = sm.execute()
    print("Outcome: " + outcome)

    # Wait for ctrl-c to stop the application
    rospy.spin()
    sis.stop()


if __name__ == '__main__':
    main()

