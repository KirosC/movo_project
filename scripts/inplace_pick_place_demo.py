#!/usr/bin/env python

import copy
import actionlib
import rospy
import tf

from moveit_msgs.msg import Constraints, OrientationConstraint

from moveit_python import (MoveGroupInterface,
                           PlanningSceneInterface,
                           PickPlaceInterface)
from moveit_python.geometry import rotate_pose_msg_by_euler_angles

from movo_action_clients.gripper_action_client import GripperActionClient
from movo_action_clients.move_base_action_client import MoveBaseActionClient

from control_msgs.msg import PointHeadAction, PointHeadGoal
from grasp_msgs.msg import FindGraspableObjectsAction, FindGraspableObjectsGoal
from shape_msgs.msg import SolidPrimitive as sp
from geometry_msgs.msg import Pose2D, PoseStamped
from moveit_msgs.msg import PlaceLocation, MoveItErrorCodes
from std_msgs.msg import Bool
from std_srvs.srv import Empty


# Point the head using controller
class PointHeadClient(object):

    def __init__(self):
        self.client = actionlib.SimpleActionClient("movo/head_controller/point_head_action", PointHeadAction)
        rospy.loginfo("Waiting for head_controller...")
        self.client.wait_for_server()
        self.pag = PointHeadGoal()
        self.pag.max_velocity = 1.0
        self.pag.pointing_axis.x = 1.0
        self.pag.pointing_frame = "/kinect2_link"

    def look_at(self, x, y, z, frame, duration=1.0):
        self.pag.target.header.stamp = rospy.get_rostime()
        self.pag.target.header.frame_id = frame
        self.pag.target.point.x = x
        self.pag.target.point.y = y
        self.pag.target.point.z = z
        self.pag.min_duration = rospy.Duration(duration)
        self.client.send_goal(self.pag)
        self.client.wait_for_result()


# Tools for grasping
class GraspingClient(object):

    def __init__(self, sim=False):
        self.scene = PlanningSceneInterface("base_link")
        self.dof = rospy.get_param('~jaco_dof')
        self.move_group = MoveGroupInterface("upper_body", "base_link")
        self.lmove_group = MoveGroupInterface("left_arm", "base_link")
        self.rmove_group = MoveGroupInterface("right_arm", "base_link")
        self.move_group.setPlannerId("RRTConnectkConfigDefault")
        self.lmove_group.setPlannerId("RRTConnectkConfigDefault")
        self.rmove_group.setPlannerId("RRTConnectkConfigDefault")

        if "7dof" == self.dof:
            self._upper_body_joints = ["right_shoulder_pan_joint",
                                       "right_shoulder_lift_joint",
                                       "right_arm_half_joint",
                                       "right_elbow_joint",
                                       "right_wrist_spherical_1_joint",
                                       "right_wrist_spherical_2_joint",
                                       "right_wrist_3_joint",
                                       "left_shoulder_pan_joint",
                                       "left_shoulder_lift_joint",
                                       "left_arm_half_joint",
                                       "left_elbow_joint",
                                       "left_wrist_spherical_1_joint",
                                       "left_wrist_spherical_2_joint",
                                       "left_wrist_3_joint",
                                       "linear_joint",
                                       "pan_joint",
                                       "tilt_joint"]
            self._right_arm_joints = ["right_shoulder_pan_joint",
                                      "right_shoulder_lift_joint",
                                      "right_arm_half_joint",
                                      "right_elbow_joint",
                                      "right_wrist_spherical_1_joint",
                                      "right_wrist_spherical_2_joint",
                                      "right_wrist_3_joint"]
            self._left_arm_joints = ["left_shoulder_pan_joint",
                                     "left_shoulder_lift_joint",
                                     "left_arm_half_joint",
                                     "left_elbow_joint",
                                     "left_wrist_spherical_1_joint",
                                     "left_wrist_spherical_2_joint",
                                     "left_wrist_3_joint"]
            self.tucked = [-1.6, -1.5, 0.4, -2.7, 0.0, 0.5, -1.7, 1.6, 1.5, -0.4, 2.7, 0.0, -0.5, 1.7, 0.04, 0, 0]
            # Pose for planning
            self.constrained_stow = [-2.6, 2.0, 0.0, 2.0, 0.0, 0.0, 1.0, 2.6, -2.0, 0.0, -2.0, 0.0, 0.0, -1.0, 0.42, 0,
                                     0]
            self.larm_const_stow = [2.6, -2.0, 0.0, -2.0, 0.0, 0.0, 1.0]
            self.rarm_const_stow = [-2.6, 2.0, 0.0, 2.0, 0.0, 0.0, -1.0]
            self.tableDist = 0.8

        else:
            rospy.logerr("DoF needs to be set 7, aborting demo")
            return

        self.pickplace = [None] * 2
        self.pickplace[0] = PickPlaceInterface("left_side", "left_gripper", verbose=True)
        self.pickplace[0].planner_id = "RRTConnectkConfigDefault"
        self.pickplace[1] = PickPlaceInterface("right_side", "right_gripper", verbose=True)
        self.pickplace[1].planner_id = "RRTConnectkConfigDefault"
        self.pick_result = [None] * 2
        self._last_gripper_picked = 0
        self.place_result = [None] * 2
        self._last_gripper_placed = 0

        self._objs_to_keep = []

        self._listener = tf.TransformListener()

        self._lgripper = GripperActionClient('left')
        self._rgripper = GripperActionClient('right')

        find_topic = "basic_grasping_perception/find_objects"
        rospy.loginfo("Waiting for %s..." % find_topic)
        self.find_client = actionlib.SimpleActionClient(find_topic, FindGraspableObjectsAction)
        self.find_client.wait_for_server()

        self.scene.clear()

        # This is a simulation so need to adjust gripper parameters
        if sim:
            self._gripper_closed = 0.96
            self._gripper_open = 0.00
        else:
            self._gripper_closed = 0.01
            self._gripper_open = 0.165

    def add_objects_to_keep(self, obj):
        self._objs_to_keep.append(obj)

    def clearScene(self):
        self.scene.clear()

    def getPickCoordinates(self):

        self.updateScene(0, False)
        pringles, grasps = self.getGraspablePringles(False)
        if None == pringles:
            return None
        center_objects = pringles.primitive_poses[0].position.y

        # surface = self.getSupportSurface(beer.support_surface)
        # tmp1 = surface.primitive_poses[0].position.x - surface.primitives[0].dimensions[0] / 2
        surface = self.getSupportSurface(pringles.support_surface)
        tmp2 = surface.primitive_poses[0].position.x - surface.primitives[0].dimensions[0] / 2
        front_edge = tmp2 - 0.1

        # Coordinate for grasping
        coords = Pose2D(x=(front_edge - self.tableDist), y=center_objects, theta=0.0)

        return coords

    def updateScene(self, gripper=0, plan=True):
        # find objects
        goal = FindGraspableObjectsGoal()
        goal.plan_grasps = plan
        goal.gripper = gripper
        self.find_client.send_goal(goal)
        self.find_client.wait_for_result(rospy.Duration(5.0))
        find_result = self.find_client.get_result()

        # remove previous objects
        for name in self.scene.getKnownCollisionObjects():
            self.scene.removeCollisionObject(name, True)

        # insert objects to scene
        idx = -1
        for obj in find_result.objects:
            if obj.object.primitive_poses[0].position.z < 0.5 or obj.object.primitive_poses[0].position.x > 2.0 or \
                    obj.object.primitive_poses[0].position.y > 0.5:
                continue
            idx += 1
            obj.object.name = "object%d_%d" % (idx, gripper)
            self.scene.addSolidPrimitive(obj.object.name,
                                         obj.object.primitives[0],
                                         obj.object.primitive_poses[0],
                                         wait=True)

        for obj in find_result.support_surfaces:
            # extend surface to floor, and make wider since we have narrow field of view
            if obj.primitive_poses[0].position.z < 0.5:
                continue
            height = obj.primitive_poses[0].position.z
            obj.primitives[0].dimensions = [obj.primitives[0].dimensions[0],
                                            obj.primitives[0].dimensions[1],  # wider
                                            obj.primitives[0].dimensions[2] + height]
            obj.primitive_poses[0].position.z += -height / 2.0

            # add to scene
            self.scene.addSolidPrimitive(obj.name,
                                         obj.primitives[0],
                                         obj.primitive_poses[0],
                                         wait=True)

        self.scene.waitForSync()

        # store for grasping
        self.objects = find_result.objects
        self.surfaces = find_result.support_surfaces

    def getGraspablePringles(self, planned=True, place=False):
        rospy.logwarn('Detected Obj: {}'.format(len(self.objects)))
        rospy.logwarn('=====================')
        for idx, obj in enumerate(self.objects):
            # need grasps
            if len(obj.grasps) < 1 and planned:
                rospy.logwarn('Ungraspable')
                rospy.logwarn('========= {} ========='.format(idx))
                continue

            # Skip pringles on the right
            if obj.object.primitive_poses[0].position.y < 0 and not place:
                continue

            # has to be on table
            if obj.object.primitive_poses[0].position.z < 0.5 or \
                    obj.object.primitive_poses[0].position.z > 1.0 or \
                    obj.object.primitive_poses[0].position.x > 2.0:
                rospy.logwarn('Not on table')
                rospy.logwarn('========= {} ========='.format(idx))
                continue
            elif obj.object.primitives[0].type == sp.CYLINDER:
                rospy.logwarn('Cylinder')
                if obj.object.primitives[0].dimensions[sp.CYLINDER_HEIGHT] < 0.18 or \
                        obj.object.primitives[0].dimensions[sp.CYLINDER_HEIGHT] > 0.28:
                    rospy.logwarn('Size not matched')
                    rospy.logwarn('========= {} ========='.format(idx))
                    continue
            elif obj.object.primitives[0].type == sp.BOX:
                rospy.logwarn('Box')
                if obj.object.primitives[0].dimensions[sp.BOX_Z] < 0.18 or \
                        obj.object.primitives[0].dimensions[sp.BOX_Z] > 0.28:
                    rospy.logwarn('Size not matched')
                    rospy.logwarn('========= {} ========='.format(idx))
                    continue
            else:
                rospy.logwarn('Unknown shape')
                rospy.logwarn('========= {} ========='.format(idx))
                continue

            rospy.logwarn('Pringles found')
            rospy.logwarn('========= {} ========='.format(idx))
            return obj.object, obj.grasps
        # nothing detected
        return None, None

    def getSupportSurface(self, name):
        for surface in self.surfaces:
            if surface.name == name:
                return surface
        return None

    def getPlaceLocation(self):
        pass

    # Block = obj, grasps = path?, gripper = arm
    def pick(self, block, grasps, gripper=0):

        success, pick_result = self.pickplace[gripper].pick_with_retry(block.name,
                                                                       grasps,
                                                                       retries=1,
                                                                       support_name=block.support_surface,
                                                                       scene=self.scene)
        self.pick_result[gripper] = pick_result
        self._last_gripper_picked = gripper
        return success

    def place(self, block, pose_stamped, gripper=0, grasps=None):
        places = list()
        loc = PlaceLocation()
        loc.place_pose.pose = pose_stamped.pose
        loc.place_pose.header.frame_id = pose_stamped.header.frame_id

        # copy the posture, approach and retreat from the grasp used
        loc.post_place_posture = grasps.pre_grasp_posture
        loc.pre_place_approach = grasps.pre_grasp_approach
        loc.post_place_retreat = grasps.post_grasp_retreat
        places.append(copy.deepcopy(loc))
        # create another several places, rotate each by 360/m degrees in yaw direction
        m = 16  # number of possible place poses
        pi = 3.141592653589
        for i in range(0, m - 1):
            loc.place_pose.pose = rotate_pose_msg_by_euler_angles(loc.place_pose.pose, 0, 0, 2 * pi / m)
            places.append(copy.deepcopy(loc))

        success, place_result = self.pickplace[gripper].place_with_retry(block.name,
                                                                         places,
                                                                         retries=1,
                                                                         scene=self.scene)
        self.place_result[gripper] = place_result
        self._last_gripper_placed = gripper
        return success

    def goto_tuck(self):
        # remove previous objects
        while not rospy.is_shutdown():
            result = self.move_group.moveToJointPosition(self._upper_body_joints, self.tucked, 0.05)
            if result.error_code.val == MoveItErrorCodes.SUCCESS:
                return

    def goto_plan_grasp(self):
        while not rospy.is_shutdown():
            result = self.move_group.moveToJointPosition(self._upper_body_joints, self.constrained_stow, 0.05)
            if result.error_code.val == MoveItErrorCodes.SUCCESS:
                return

    def left_arm_constrained_stow(self):
        c1 = Constraints()
        c1.orientation_constraints.append(OrientationConstraint())
        c1.orientation_constraints[0].header.stamp = rospy.get_rostime()
        c1.orientation_constraints[0].header.frame_id = "base_link"
        c1.orientation_constraints[0].link_name = "left_ee_link"
        c1.orientation_constraints[0].orientation.w = 1.0
        c1.orientation_constraints[0].absolute_x_axis_tolerance = 0.2  # x axis is pointed up for wrist link
        c1.orientation_constraints[0].absolute_y_axis_tolerance = 0.2
        c1.orientation_constraints[0].absolute_z_axis_tolerance = 6.28
        c1.orientation_constraints[0].weight = 1.0

        while not rospy.is_shutdown():
            result = self.lmove_group.moveToJointPosition(self._left_arm_joints, self.larm_const_stow, 0.05,
                                                          path_constraints=c1, planning_time=120.0)
            if result.error_code.val == MoveItErrorCodes.SUCCESS:
                return

    def right_arm_constrained_stow(self):
        c1 = Constraints()
        c1.orientation_constraints.append(OrientationConstraint())
        c1.orientation_constraints[0].header.stamp = rospy.get_rostime()
        c1.orientation_constraints[0].header.frame_id = "base_link"
        c1.orientation_constraints[0].link_name = "right_ee_link"
        c1.orientation_constraints[0].orientation.w = 1.0
        c1.orientation_constraints[0].absolute_x_axis_tolerance = 0.2  # x axis is pointed up for wrist link
        c1.orientation_constraints[0].absolute_y_axis_tolerance = 0.2
        c1.orientation_constraints[0].absolute_z_axis_tolerance = 6.28
        c1.orientation_constraints[0].weight = 1.0

        while not rospy.is_shutdown():
            result = self.rmove_group.moveToJointPosition(self._right_arm_joints, None, 0.05,
                                                          path_constraints=c1, planning_time=120.0)
            if result.error_code.val == MoveItErrorCodes.SUCCESS:
                return

    def open_gripper(self):
        self._lgripper.command(self._gripper_open, block=True)
        self._rgripper.command(self._gripper_open, block=True)

    def close_gripper(self):
        self._lgripper.command(self._gripper_closed, block=True)
        self._rgripper.command(self._gripper_closed, block=True)


def convert_dict_to_pose2d(loc):
    tmp = Pose2D(loc["x"], loc["y"], loc["theta"])
    return tmp


def goto_origin(req):
    rospy.loginfo("Moving to origin...")
    target = Pose2D(x=0.0, y=0.0, theta=0.0)
    move_base.goto(target)
    rospy.loginfo("Moving is complete...")


def tuck(req):
    rospy.loginfo("Transforming to tuck position")
    grasping_client.close_gripper()
    grasping_client.goto_tuck()
    rospy.loginfo("Transforming is complete...")


def grasp_pos(req):
    rospy.loginfo("Transforming to grasping position")
    grasping_client.goto_plan_grasp()
    grasping_client.open_gripper()
    rospy.loginfo("Transforming is complete...")


if __name__ == "__main__":

    # Create a node
    rospy.init_node("inplace_pick_place_demo")

    """
    Get all the demo locations from the parameter file defined by the user
    """
    table_height = rospy.get_param("~table_height", 0.75)

    is_sim = rospy.get_param("~sim", False)

    if is_sim:
        rospy.wait_for_message('/sim_initialized', Bool)

    # Setup clients
    move_base = MoveBaseActionClient(sim=is_sim, frame="odom")
    head_action = PointHeadClient()
    grasping_client = GraspingClient(sim=is_sim)
    grasping_client.clearScene()

    # Setup utils service server
    rospy.Service('util_goto_origin', Empty, goto_origin)
    rospy.Service('util_tuck', Empty, tuck)
    rospy.Service('util_grasp_pos', Empty, grasp_pos)

    # set the robot pos to the planning pose
    grasping_client.goto_tuck()
    grasping_client.close_gripper()
    grasping_client.goto_plan_grasp()
    grasping_client.open_gripper()

    # 1.2 is the straight distance between the robot and object
    head_action.look_at(2.0, 0, table_height + .1, "base_link")

    while not rospy.is_shutdown():
        coords = grasping_client.getPickCoordinates()
        if coords is None:
            rospy.logwarn("Perception failed.")
            continue
        break

    rospy.loginfo("Moving to table...")
    rospy.logwarn('Coords: \n{}'.format(coords))
    move_base.goto(coords)

    # Point the head at the stuff we want to pick
    head_action.look_at(0.9, 0.0, table_height + .1, "base_link")
    while not rospy.is_shutdown():
        rospy.loginfo("Picking object...")
        # Update scene for specific arm
        grasping_client.updateScene(0)
        pringles, grasps = grasping_client.getGraspablePringles()
        if pringles == None:
            rospy.logwarn("Perception failed.")
            continue

        # Pick the pringles
        if grasping_client.pick(pringles, grasps, 0):
            break
        rospy.logwarn("Grasping failed.")

    # Goto grasping position
    grasping_client.goto_plan_grasp()

    # Point the head at the stuff we want to pick
    head_action.look_at(0.9, 0.0, table_height + .1, "base_link")

    # Place the block
    while not rospy.is_shutdown():
        rospy.loginfo("Scanning another pringles...")

        # Scanning for the pringles on the table
        grasping_client.updateScene(0)
        pringles_right, graspsRight = grasping_client.getGraspablePringles(place=True)
        if pringles_right == None:
            rospy.logwarn("Perception failed.")
            continue

        rospy.loginfo("Placing object...")
        pose = PoseStamped()
        pose.pose = pringles_right.primitive_poses[0]
        # Place above another pringles, pringles height approx. = 0.27
        if pringles_right.primitives[0].type == sp.CYLINDER:
            pringles_height = pringles_right.primitives[0].dimensions[sp.CYLINDER_HEIGHT]
        else:
            pringles_height = pringles_right.primitives[0].dimensions[sp.BOX_Z]
        rospy.logwarn('Pringle height: {}'.format(pringles_height))
        pose.pose.position.z += pringles_height + 0.01
        # Fine tune for pringles position
        pose.pose.position.x -= 0.05
        pose.header.frame_id = pringles_right.header.frame_id

        bestGrasp = graspsRight[0]
        for grasp in graspsRight:
            if grasp.grasp_quality > bestGrasp:
                bestGrasp = grasp

        if grasping_client.place(pringles, pose, gripper=0, grasps=bestGrasp):
            break
        rospy.logwarn("Placing failed.")

    grasping_client.goto_plan_grasp()

    # Demo finished return to tuck
    grasping_client.close_gripper()
    grasping_client.goto_tuck()

    rospy.loginfo("Moving to start...")
    target = Pose2D(x=0.0, y=0.0, theta=0.0)
    move_base.goto(target)

    rospy.loginfo("Demo is complete...")
