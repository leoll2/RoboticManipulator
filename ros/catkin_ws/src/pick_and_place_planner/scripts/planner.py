#!/usr/bin/env python

import rosgraph
import rospy
import smach
import smach_ros
from actionlib_msgs.msg import GoalStatus
from geometry_msgs.msg import Pose, Quaternion
from tf.transformations import quaternion_from_euler

from pick_and_place_planner.msg import \
    PickAction, PickGoal, PickResult, PickFeedback, \
    PlaceAction, PlaceGoal, PlaceResult, PlaceFeedback, \
    RestAction, RestGoal, RestResult, RestFeedback


class Block:

    def __init__(self, start_pose, end_pose):

        self.start_pose = Pose()
        self.start_pose.position.x = start_pose[0]
        self.start_pose.position.y = start_pose[1]
        self.start_pose.position.z = start_pose[2]
        self.start_pose.orientation = Quaternion(
            *quaternion_from_euler(start_pose[3], start_pose[4], start_pose[5]))
        self.end_pose = Pose()
        self.end_pose.position.x = end_pose[0]
        self.end_pose.position.y = end_pose[1]
        self.end_pose.position.z = end_pose[2]
        self.end_pose.orientation = Quaternion(
            *quaternion_from_euler(end_pose[3], end_pose[4], end_pose[5]))


class BlockFactory:

    def __init__(self):

        self.blocks = (
            Block([-0.3, 0.3, 0.2, 0.0, 0.0, 0.0], [-0.15, -0.45, 0.075, 0.0, 0.0, 0.0]),
            Block([0.4, 0.1, 0.15, 0.0, 0.0, 0.0], [0, -0.45, 0.075, 0.0, 0.0, 0.0]),
            Block([0.4, 0.2, 0.2, 0.0, 0.0, 0.0], [-0, -0.3, 0.075, 0.0, 0.0, 0.0]),
            Block([-0.4, 0.3, 0.15, 0.0, 0.0, 0.0], [-0.15, -0.3, 0.075, 0.0, 0.0, 0.0]),
            Block([0, 0.4, 0.3, 0.0, 0.0, 0.0], [-0.075, -0.375, 0.225, 0.0, 0.0, 0.0])
        )
        self.remaining_blocks = len(self.blocks)

    def get_block(self):

        if self.remaining_blocks > 0:
            self.remaining_blocks -= 1
            return self.blocks[len(self.blocks) - self.remaining_blocks - 1]
        else:
            return None


# 'SELECT' state
class Select(smach.State):

    def __init__(self, block_factory):
        smach.State.__init__(self, outcomes=['selected', 'out_of_blocks'],
                             output_keys=['start_pose', 'end_pose'])
        self.block_factory = block_factory

    def execute(self, userdata):
        rospy.loginfo('Executing state: Select')
        block = self.block_factory.get_block()
        if block is not None:
            userdata.start_pose = block.start_pose
            userdata.end_pose = block.end_pose
            return 'selected'
        else:
            return 'out_of_blocks'


def main():

    if not rosgraph.is_master_online():
        print('Error: ROS master not running')
        exit(1)

    rospy.init_node('pick_and_place_state_machine')
    bf = BlockFactory()

    # Create a SMACH state machine
    sm = smach.StateMachine(outcomes=['finished', 'failed'])

    # Open the container
    with sm:
        # Add states
        smach.StateMachine.add('SELECT', Select(bf),
                               transitions={'selected': 'PICKANDPLACE',
                                            'out_of_blocks': 'REST'})

        # Submachine for pick-and-place
        sm_pickplace = smach.StateMachine(outcomes=['placed', 'failed'],
                                          input_keys=['start_pose', 'end_pose'])
        with sm_pickplace:
            def pick_goal_cb(userdata, goal):
                pick_goal = PickGoal()
                pick_goal.obj_pose = userdata.start_pose
                return pick_goal

            def pick_result_cb(userdata, status, result):
                if status == GoalStatus.SUCCEEDED:
                    userdata.target_pose = userdata.end_pose
                    return 'succeeded'
                elif status == GoalStatus.PREEMPTED:
                    return 'preempted'
                else:
                    return 'aborted'

            smach.StateMachine.add('PICK',
                                   smach_ros.SimpleActionState(
                                       'pick_action_server',
                                       PickAction,
                                       goal_cb=pick_goal_cb,
                                       result_cb=pick_result_cb,
                                       input_keys=['start_pose', 'end_pose'],
                                       output_keys=['target_pose']
                                   ),
                                   transitions={'succeeded': 'PLACE',
                                                'preempted': 'failed',
                                                'aborted': 'failed'})

            def place_goal_cb(userdata, goal):
                place_goal = PlaceGoal()
                place_goal.target_pose = userdata.target_pose
                return place_goal

            smach.StateMachine.add('PLACE',
                                   smach_ros.SimpleActionState(
                                       'place_action_server',
                                       PlaceAction,
                                       goal_cb=place_goal_cb,
                                       input_keys=['target_pose']),
                                   transitions={'succeeded': 'placed',
                                                'preempted': 'failed',
                                                'aborted': 'failed'})
        smach.StateMachine.add('PICKANDPLACE', sm_pickplace,
                               transitions={'placed': 'SELECT',
                                            'failed': 'failed'})

        def rest_goal_cb(userdata, goal):
            rest_goal = RestGoal()
            return rest_goal

        smach.StateMachine.add('REST',
                               smach_ros.SimpleActionState(
                                   'rest_action_server',
                                   RestAction,
                                   goal_cb=rest_goal_cb),
                               transitions={'succeeded': 'finished',
                                            'preempted': 'failed',
                                            'aborted': 'failed'})

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

