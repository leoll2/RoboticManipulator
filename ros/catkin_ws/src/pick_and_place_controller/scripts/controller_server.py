#! /usr/bin/env python


import actionlib
import math
import numpy as np
import rosgraph
import rospy
import time
from tqdm import tqdm
from geometry_msgs.msg import Pose
from std_msgs.msg import String
from pick_and_place_controller.msg import \
     PickAction, PickGoal, PickResult, PickFeedback, \
     PlaceAction, PlaceGoal, PlaceResult, PlaceFeedback, \
     RestAction, RestGoal, RestResult, RestFeedback


class PickActionServer:

    def __init__(self):

        self.last_ee_pose = Pose()
        self.last_ee_pose.position.x = self.last_ee_pose.position.y = self.last_ee_pose.position.z = 1000  # unreachable
        self.obj_pose = Pose()
        self.obj_pose.position.x = self.obj_pose.position.y = self.obj_pose.position.z = 0  # default very far from ee
        self.max_seconds = 60
        self.active = False
        self.pbar_x = self.pbar_y = self.pbar_z = None

        self.a_server = actionlib.SimpleActionServer(
            "pick_action_server",
            PickAction,
            execute_cb=self.execute_cb,
            auto_start=False)

        self.a_server.start()
        rospy.loginfo("Started PickActionServer")

        self.sub = rospy.Subscriber("robot/end_effector/pose", Pose, callback=self.ee_pose_cb)
        self.pose_pub = rospy.Publisher('robot/end_effector/target_pose', Pose, queue_size=1)
        self.mode_pub = rospy.Publisher('robot/opmode', String, queue_size=1)

    def object_nearby(self):
        """ Returns True if the end effector is very close to the object, False otherwise """

        ee_pos = np.array((self.last_ee_pose.position.x, self.last_ee_pose.position.y, self.last_ee_pose.position.z))
        obj_pos = np.array((self.obj_pose.position.x, self.obj_pose.position.y, self.obj_pose.position.z))
        dist = np.linalg.norm(ee_pos - obj_pos)
        # print("\rDistance: %.3f" % dist)
        self.pbar_x.n = math.floor(100 * (1 - abs(ee_pos[0] - obj_pos[0]))) if abs(ee_pos[0] - obj_pos[0]) < 1 else 0
        self.pbar_y.n = math.floor(100 * (1 - abs(ee_pos[1] - obj_pos[1]))) if abs(ee_pos[1] - obj_pos[1]) < 1 else 0
        self.pbar_z.n = math.floor(100 * (1 - abs(ee_pos[2] - obj_pos[2]))) if abs(ee_pos[2] - obj_pos[2]) < 1 else 0
        self.pbar_x.refresh()
        self.pbar_y.refresh()
        self.pbar_z.refresh()

        return dist < 0.02

    def ee_pose_cb(self, msg):

        if not self.active:
            return
        # rospy.loginfo("PickActionServer: received EE pose update: " + str(msg.position))
        self.last_ee_pose = msg

    def execute_cb(self, goal):

        rospy.loginfo("Going to pick object located at (%.3f, %.3f, %.3f)..." %
                      (goal.obj_pose.position.x, goal.obj_pose.position.y, goal.obj_pose.position.z))
        self.obj_pose = goal.obj_pose
        self.active = True
        self.pose_pub.publish(goal.obj_pose)
        self.mode_pub.publish('pick')
        self.pbar_x = tqdm(range(100), position=0, desc="X alignment")
        self.pbar_y = tqdm(range(100), position=1, desc="Y alignment")
        self.pbar_z = tqdm(range(100), position=2, desc="Z alignment")

        result = PickResult()
        rate = rospy.Rate(1)
        elapsed = 0
        success = False
        while elapsed < self.max_seconds and not rospy.is_shutdown():
            # Check if preempted (if so, abort)
            if self.a_server.is_preempt_requested():
                self.a_server.set_preempted()
                self.active = False
                return
            elif self.object_nearby():
                print('\n\n')
                rospy.loginfo("...picked!")
                success = True
                break
            else:
                # TODO publish feedback
                pass
            elapsed += 1
            rate.sleep()

        self.active = False
        if success:
            self.a_server.set_succeeded(result)
        else:
            print("Aborted picking")
            self.a_server.set_aborted()


class PlaceActionServer:

    def __init__(self):

        self.last_ee_pose = Pose()
        self.last_ee_pose.position.x = self.last_ee_pose.position.y = self.last_ee_pose.position.z = 1000  # unreachable
        self.dest_pose = Pose()
        self.dest_pose.position.x = self.dest_pose.position.y = self.dest_pose.position.z = 0  # default very far from ee
        self.active = False
        self.max_seconds = 60
        self.pbar_x = self.pbar_y = self.pbar_z = None

        self.a_server = actionlib.SimpleActionServer(
            "place_action_server",
            PlaceAction,
            execute_cb=self.execute_cb,
            auto_start=False)

        self.a_server.start()
        rospy.loginfo("Started PlaceActionServer")

        self.sub = rospy.Subscriber("robot/end_effector/pose", Pose, callback=self.ee_pose_cb)
        self.pose_pub = rospy.Publisher('robot/end_effector/target_pose', Pose, queue_size=1)
        self.mode_pub = rospy.Publisher('robot/opmode', String, queue_size=1)

    def destination_nearby(self):
        """ Returns True if the end effector is very close to the destination, False otherwise """

        ee_pos = np.array((self.last_ee_pose.position.x, self.last_ee_pose.position.y, self.last_ee_pose.position.z))
        dest_pos = np.array((self.dest_pose.position.x, self.dest_pose.position.y, self.dest_pose.position.z))
        dist = np.linalg.norm(ee_pos - dest_pos)
        # print("\rDistance: %.3f" % dist)
        self.pbar_x.n = math.floor(100 * (1 - abs(ee_pos[0] - dest_pos[0]))) if abs(ee_pos[0] - dest_pos[0]) < 1 else 0
        self.pbar_y.n = math.floor(100 * (1 - abs(ee_pos[1] - dest_pos[1]))) if abs(ee_pos[1] - dest_pos[1]) < 1 else 0
        self.pbar_z.n = math.floor(100 * (1 - abs(ee_pos[2] - dest_pos[2]))) if abs(ee_pos[2] - dest_pos[2]) < 1 else 0
        self.pbar_x.refresh()
        self.pbar_y.refresh()
        self.pbar_z.refresh()
        return dist < 0.01

    def ee_pose_cb(self, msg):

        if not self.active:
            return
        self.last_ee_pose = msg

    def execute_cb(self, goal):

        rospy.loginfo("Going to place object to (%.3f, %.3f, %.3f)..." %
                      (goal.target_pose.position.x, goal.target_pose.position.y, goal.target_pose.position.z))
        self.dest_pose = goal.target_pose
        self.active = True
        self.pose_pub.publish(goal.target_pose)
        self.mode_pub.publish('place')
        self.pbar_x = tqdm(range(100), position=0, desc="X alignment")
        self.pbar_y = tqdm(range(100), position=1, desc="Y alignment")
        self.pbar_z = tqdm(range(100), position=2, desc="Z alignment")

        result = PlaceResult()
        rate = rospy.Rate(1)
        elapsed = 0
        success = False
        while elapsed < self.max_seconds and not rospy.is_shutdown():
            # Check if preempted (if so, abort)
            if self.a_server.is_preempt_requested():
                self.a_server.set_preempted()
                self.active = False
                return
            elif self.destination_nearby():
                print('\n\n')
                rospy.loginfo("...placed!")
                success = True
                break
            else:
                # TODO publish feedback
                pass
            elapsed += 1
            rate.sleep()

        self.active = False
        if success:
            self.a_server.set_succeeded(result)
        else:
            print("Aborted placing")
            self.a_server.set_aborted()


class RestActionServer:

    def __init__(self):

        self.a_server = actionlib.SimpleActionServer(
            "rest_action_server",
            RestAction,
            execute_cb=self.execute_cb,
            auto_start=False)

        self.a_server.start()
        rospy.loginfo("Started RestActionServer")

        self.mode_pub = rospy.Publisher('robot/opmode', String, queue_size=1)

    def execute_cb(self, goal):

        rospy.loginfo("Returning to resting position")
        self.mode_pub.publish('rest')
        result = PlaceResult()
        self.a_server.set_succeeded(result)


if __name__ == "__main__":

    if not rosgraph.is_master_online():
        print('Error: ROS master not running')
        exit(1)

    rospy.init_node("pick_and_place_action_server")

    pick_act_srv = PickActionServer()
    place_act_srv = PlaceActionServer()
    rest_act_srv = RestActionServer()

    rospy.spin()

