#!/usr/bin/env python

import rospy
import tf2_geometry_msgs
import tf2_ros
from geometry_msgs.msg import Twist, PoseStamped
from nav_msgs.msg import Odometry, Path
from nav_msgs.srv import GetPlan, GetPlanResponse
import numpy as np
from tf.transformations import euler_from_quaternion
from utils import normalize_theta


class PathFollower(object):
  def __init__(self, max_vel_x = 1.3, max_vel_y = 1.3, max_vel_theta = 2.0, acc_x=3.96, acc_y=2.0, acc_theta= 10.0, dt = 0.1, ctrl_freq=10, plan_freq=1, dist_tolerance = 0.1, orient_tolerance = 0.2):
    #Properties
    self.global_plan = Path()
    self.plan_srv = "global_planner/planner/make_plan"
    self.controller_dist_tolerance = dist_tolerance
    self.controller_orient_tolerance = orient_tolerance
    self.max_vel_x = max_vel_x
    self.max_vel_y = max_vel_y
    self.max_vel_theta = max_vel_theta
    self.acc_x = acc_x
    self.acc_y = acc_y
    self.acc_theta = acc_theta
    self.dt = dt #0.1
    self.goal_reached_ = True
    self.got_plan = False
    self.goal = PoseStamped()
    self.current_pose = PoseStamped()
    self.current_vel = Twist()
    self.odom_pose = PoseStamped()
    self.feedback_topic = "odom"


    rospy.init_node('PathFollowerROS')
    self.cmd_pub = rospy.Publisher("/cmd_vel", Twist, queue_size = 10)
    feedback_sub_ = rospy.Subscriber(self.feedback_topic, Odometry, self.updatePoseVel)
    goal_sub_ = rospy.Subscriber("/goal", PoseStamped, self.updateGoal)
    rospy.Timer(rospy.Duration(1.0/plan_freq), self.getGlobalPlan) ## check this
    self.tf = tf2_ros.Buffer()
    self.listener = tf2_ros.TransformListener(self.tf)

    rate =  rospy.Rate(ctrl_freq)
    while not rospy.is_shutdown():
      if not self.goal_reached_ and self.got_plan:
        self.publishCommandVelocity()
      rate.sleep()

  def updateGoal(self, msg):
    self.goal = msg
    self.goal_reached_ = False

  def getGlobalPlan(self, event):
    if self.goal_reached_:
      self.got_plan = False
      return

    rospy.wait_for_service(self.plan_srv)
    try:
      get_plan_srv = rospy.ServiceProxy(self.plan_srv, GetPlan)
      # srv_ = GetPlan()
      # srv_.start = self.current_pose
      # srv_.goal = self.goal
      # srv_.tolerance = 0.1
      res = get_plan_srv(self.current_pose, self.goal, 0.1)
      if(len(res.plan.poses)<3):
        return
      self.global_plan = res.plan
      self.got_plan = True
    except rospy.ServiceException as e:
      print("Service call failed: %s"%e)

  def updatePoseVel(self, msg):
    pose_ = PoseStamped()
    pose_.header = msg.header
    pose_.pose = msg.pose.pose
    ## Need to update transform them to map frame
    point_transform = self.tf.lookup_transform("map", "odom", rospy.Time(0),rospy.Duration(0.3))
    self.current_pose = tf2_geometry_msgs.do_transform_pose(pose_, point_transform)
    self.odom_pose = pose_
    self.current_vel = msg.twist.twist
    # print(self.current_pose)

  def publishCommandVelocity(self):

    #### Have to transform pose to odom frame
    global_goal = self.global_plan.poses[-1]

    rot = global_goal.pose.orientation
    _,_,yaw_goal = euler_from_quaternion([rot.x, rot.y, rot.z, rot.w])
    rot = self.current_pose.pose.orientation
    _,_,yaw_current = euler_from_quaternion([rot.x, rot.y, rot.z, rot.w])

    dx = (global_goal.pose.position.x - self.current_pose.pose.position.x)
    dy = (global_goal.pose.position.y - self.current_pose.pose.position.y)
    d_omega = normalize_theta(yaw_goal-yaw_current)

    if np.linalg.norm([dx,dy]) <= self.controller_dist_tolerance and abs(d_omega) <= self.controller_orient_tolerance:
      self.goal_reached_ = True
      return


    cmd_vel = self.calculateCommandVelocity()
    self.cmd_pub.publish(cmd_vel)

  def transformGlobalPlan(self):
    g_plan = self.global_plan.poses
    trfm_plan = Path()
    trfm_plan.header.frame_id = "odom"

    point_transform = self.tf.lookup_transform("odom", "map", rospy.Time(0),rospy.Duration(0.3))

    for i in range(0,len(g_plan)):
      pose = tf2_geometry_msgs.do_transform_pose(g_plan[i], point_transform)
      trfm_plan.poses.append(pose)

    return trfm_plan


  def calculateCommandVelocity(self):
    trfm_plan = self.transformGlobalPlan()
    g_plan = trfm_plan.poses
    print(g_plan[0].header.frame_id)
    pos = self.odom_pose
    vel = self.current_vel

    rot = pos.pose.orientation
    _,_,yaw_current = euler_from_quaternion([rot.x, rot.y, rot.z, rot.w])

    idx = 0
    vx =[]
    vy =[]
    v_ang = []

    for i in range(0,len(g_plan)):
      vx.append((g_plan[i].pose.position.x - pos.pose.position.x)/self.dt)
      vy.append((g_plan[i].pose.position.y - pos.pose.position.y)/self.dt)

      rot = g_plan[i].pose.orientation
      _,_,yaw_goal = euler_from_quaternion([rot.x, rot.y, rot.z, rot.w])

      v_ang.append(normalize_theta(yaw_goal-yaw_current)/self.dt)

      if i==1:
        idx = i
        break

      if (abs(vx[i]) >= self.max_vel_x) or (abs(vy[i]) >= self.max_vel_y) or (abs(v_ang[i]) >= self.max_vel_theta):
        idx = i-1
        break

    for i in range(idx,-1,-1):
      ax = (vx[i]-vel.linear.x)/self.dt
      ay = (vy[i]-vel.linear.y)/self.dt
      az = (v_ang[i]-vel.angular.z)/self.dt

      if abs(ax) < self.acc_x and abs(ay) < self.acc_y and abs(az) < self.acc_theta:
        idx = i
        break

    command_velocity = Twist()
    command_velocity.linear.x = vx[idx]
    command_velocity.linear.y = vy[idx]
    command_velocity.angular.z = v_ang[idx]

    return command_velocity


if __name__ == '__main__':
  ctrl = PathFollower()
