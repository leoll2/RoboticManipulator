#! /usr/bin/env python

import rospy
import actionlib
from geometry_msgs.msg import Pose
from pick_and_place_controller.msg import \
     PickAction, PickGoal, PickResult, PickFeedback, \
     PlaceAction, PlaceGoal, PlaceResult, PlaceFeedback


class PickActionServer:

    def __init__(self):

        self.a_server = actionlib.SimpleActionServer(
            "pick_action_server",
            PickAction,
            execute_cb=self.execute_cb,
            auto_start=False)

        self.a_server.register_preempt_callback(self.preemptCB)

        self.a_server.start()
        rospy.loginfo("Started PickActionServer")

        self._sub = rospy.Subscriber("/end_effector_pose", Pose, callback=self.eePoseCB)

    def eePoseCB(self, data):

        rospy.loginfo("eePoseCB")
        # TODO controlla se la posizione e' giusta e nel caso conferma un succeeded
        # success = True
        # feedback = PickFeedback()
        # result = PickResult()
        # last_dish_washed = 'bowl-' + str(i)
        # feedback.last_dish_washed = last_dish_washed
        # result.dishes_washed.append(last_dish_washed)
        # self.a_server.publish_feedback(feedback)
        # if success:
        #     self.a_server.set_succeeded(result)
        pass

    def preemptCB(self):

        rospy.loginfo("preemptCB")
        self.a_server.set_preempted()

    def execute_cb(self, goal):

        rospy.loginfo("executeCB")
        obj_pose = goal.obj_pose
        # TODO publish desired position

        result = PickResult()        # TODO this is provisional
        result.final_pose = obj_pose  # TODO this is provisional
        self.a_server.set_succeeded(result)  # TODO this is provisional


class PlaceActionServer:

    def __init__(self):

        self.a_server = actionlib.SimpleActionServer(
            "place_action_server",
            PlaceAction,
            execute_cb=self.execute_cb,
            auto_start=False)

        self.a_server.register_preempt_callback(self.preemptCB)

        self.a_server.start()
        rospy.loginfo("Started PlaceActionServer")

        self._sub = rospy.Subscriber("/end_effector_pose", Pose, callback=self.eePoseCB)

    def eePoseCB(self, data):
        # TODO
        rospy.loginfo("eePoseCB")

    def preemptCB(self):

        rospy.loginfo("preemptCB")
        self.a_server.set_preempted()

    def execute_cb(self, goal):

        rospy.loginfo("executeCB")
        target_pose = goal.target_pose
        # TODO publish desired position

        result = PlaceResult()        # TODO this is provisional
        result.final_pose = target_pose  # TODO this is provisional
        self.a_server.set_succeeded(result)  # TODO this is provisional


if __name__ == "__main__":
    rospy.init_node("action_server")

    pick_act_srv = PickActionServer()
    place_act_srv = PlaceActionServer()
    rospy.spin()
