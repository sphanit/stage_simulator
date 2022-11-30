import rospy
import rvo2
from navfn.srv import MakeNavPlan, MakeNavPlanRequest
from nav_msgs.msg import Odometry, Path
from geometry_msgs.msg import Twist, PoseStamped, PoseArray
# from visualization_msgs.msg import Marker, MarkerArray
from transformations import euler_from_quaternion
import math
import numpy as np
import message_filters

plan_srv_name_ = 'gbl_planner/make_plan'

obstacles = dict()
obstacles['open_space'] = [[0.43797940015792847, 9.517568588256836], [0.5261608958244324, 0.47761908173561096], 
                          [39.404258728027344, 0.4483940005302429], [39.387760162353516, 9.493051528930664], [21.946561813354492, 7.450645923614502], 
                          [21.90517807006836, 5.555723190307617], [23.949962615966797, 5.52982234954834], [23.896080017089844, 7.479827880859375], 
                          [9.670744895935059, 3.500570297241211], [9.62537670135498, 2.755267858505249], [35.2904052734375, 2.754443883895874], 
                          [35.2723388671875, 3.4841744899749756]]
class Human(object):
    def __init__(self, id = None):
        self.id = id
        self.x = 0
        self.y = 0
        self.theta = 0
        self.vel_x = 0
        self.vel_y = 0
        self.omega = 0
        self.pref_vel_x = 0
        self.pref_vel_y = 0
        self.pref_omega = 0
        self.goal = [None, None]
        self.goal_reached = False
    def __repr__(self):
        return repr([self.x, self.y, self.theta, self.vel_x,self.vel_y,
                    self.omega, self.pref_vel_x, self.pref_vel_y, self.pref_omega, self.goal, self.goal_reached])

class Goal(object):
    def __init__(self, id, pos):
        self.id = id
        self.goal = pos
    def __repr(self):
        return repr([self.id, self.goal])


def normalize_theta(theta):
  PI = math.pi
  result = math.fmod(theta + PI, 2.0 * PI)
  if result <= 0:
    return result + PI
  return result - PI
  
def clamp(x, minn, maxx):
   return x if x > minn and x < maxx else (minn if x < minn else maxx)

class ORCA_Sim(object):
    def __init__(self, map_name, num_hum, dt, look_ahead_poses, timestep, neighborDist, maxNeighbors, timeHorizon, timeHorizonObst, radius, maxSpeed):
        self.sim = rvo2.PyRVOSimulator(timestep, neighborDist, maxNeighbors, timeHorizon, timeHorizonObst, radius, maxSpeed)
        self.map_name = map_name
        self.num_hum = num_hum
        self.dt = dt
        self.look_ahead = look_ahead_poses 
        self.humans = []
        self.plan_pubs_ = []
        self.plans = []
        self.orca_agents = [[]]*num_hum
        self.got_init_poses = False
        for i in range(0,num_hum):
            self.humans.append(Human(i))
            pub_ = rospy.Publisher('plan_human' + str(i), PoseArray, queue_size=10)
            self.plan_pubs_.append(pub_)
        
        
        self.add_obstcles()

    def add_obstcles(self):
        lines = []
        for i in range(0,4):
            next_idx = (i+1)% 4
            # dir = [a_i - b_i for a_i, b_i in zip(obstacles[self.map_name][next_idx], obstacles[self.map_name][i])]
            line = []
            line.append(obstacles[self.map_name][i])
            line.append(obstacles[self.map_name][next_idx])
            lines.append(line)
        
        polygons = []
        polygon = []
        for i in range(4, len(obstacles[self.map_name])):
            polygon.append(obstacles[self.map_name][i])
            if i%4 == 3 and len(polygon) > 0:
                polygons.append(polygon)
                polygon = []

        for i in range(0, len(lines)):
            _ = self.sim.addObstacle(lines[i])

        for i in range(0, len(polygons)):
            _ = self.sim.addObstacle(polygons[i])

    def add_agents(self):
        for human in self.humans:
            self.orca_agents[human.id] = self.sim.addAgent((human.x, human.y))

    def humansCB(self, *humOdoms):
        for i in range(0,len(humOdoms)):
            self.humans[i].x = humOdoms[i].pose.pose.position.x
            self.humans[i].y = humOdoms[i].pose.pose.position.y
            quat = [humOdoms[i].pose.pose.orientation.x, humOdoms[i].pose.pose.orientation.y, 
                    humOdoms[i].pose.pose.orientation.z, humOdoms[i].pose.pose.orientation.w]
            _,_,yaw = euler_from_quaternion(quat)
            self.humans[i].theta = yaw
            self.humans[i].vel_x = humOdoms[i].twist.twist.linear.x
            self.humans[i].vel_y = humOdoms[i].twist.twist.linear.y
            self.humans[i].omega = humOdoms[i].twist.twist.angular.z
        
        if not self.got_init_poses:
            self.add_agents()
            print('Simulation has %i agents and %i obstacle vertices in it.' %
                    (self.sim.getNumAgents(), self.sim.getNumObstacleVertices()))
            # print(self.humans)
            self.got_init_poses = True
    
    def get_velocities(self):
        self.update_pref_velocities()
        for i in range(0,self.num_hum):
            self.sim.setAgentPrefVelocity(self.orca_agents[i], (self.humans[i].pref_vel_x, self.humans[i].pref_vel_y))
            self.sim.setAgentPosition(self.orca_agents[i], (self.humans[i].x, self.humans[i].y))
        self.sim.doStep()
        human_vels = []
        for i in range(0,self.num_hum):
            orca_vx, orca_vy = self.sim.getAgentVelocity(self.orca_agents[i])
            theta = self.humans[i].theta
            vel = Twist()
            current_alpha = abs(self.humans[i].pref_omega - self.humans[i].omega)/self.dt
            if current_alpha >= 0:
                alpha = min(current_alpha, 20.0)
            else:
                alpha = max(current_alpha, -20.0)
            vel.linear.x = orca_vx*math.cos(theta)+orca_vy*math.sin(theta)
            vel.linear.y = -orca_vx*math.sin(theta)+orca_vy*math.cos(theta)
            vel.angular.z = min(alpha*self.dt, clamp(self.humans[i].pref_omega, -2.0, 2.0)) #math.atan2(vel.linear.y, vel.linear.x)/self.dt  # 
            human_vels.append(vel)
        self.publish_plans()
        return human_vels

    def simulate_forward(self):
        #TODO
        print()

    def publish_plans(self):
        now = rospy.Time.now()
        i = 0
        for plan in self.plans:
            pose_array = PoseArray()
            pose_array.header.stamp = now
            pose_array.header.frame_id = 'map'
            for pose in plan.path:
                pose_array.poses.append(pose.pose)
            self.plan_pubs_[i].publish(pose_array)
            i = i + 1 

    def get_plans(self, get_plan_srv_):
        # print('Planning..') 
        plans = []       
        for human in self.humans:
            start = PoseStamped()
            start.header.seq = 0
            start.header.frame_id = "map"
            start.header.stamp = rospy.Time.now()
            start.pose.position.x = human.x   
            start.pose.position.y = human.y 

            Goal = PoseStamped()
            Goal.header.seq = 0
            Goal.header.frame_id = "map"
            Goal.header.stamp = rospy.Time.now()
            if human.goal[0] is None or human.goal[1] is None:
                Goal.pose.position.x = human.x
                Goal.pose.position.y = human.y
            else:
                Goal.pose.position.x = human.goal[0]  
                Goal.pose.position.y = human.goal[1] 

            r = MakeNavPlan()
            r.start = start
            r.goal = Goal
            # print(start)
            # print(Goal)
            plan = get_plan_srv_(r.start, r.goal)
            # print(len(plan.path))
            plans.append(plan)
        self.plans = plans

        if len(self.plans) == self.num_hum:
            return True
        else:
            return False

    def update_pref_velocities(self):
        for i in range(0, self.num_hum):
            if self.humans[i].goal[0] is None or self.humans[i].goal[1] is None:
                self.humans[i].pref_vel_x = 0
                self.humans[i].pref_vel_y = 0
                self.humans[i].pref_omega = 0
            elif np.linalg.norm([self.humans[i].goal[0] - self.humans[i].x, self.humans[i].goal[1] - self.humans[i].y]) < 0.2 \
                or len(self.plans[i].path) <2:
                self.humans[i].goal_reached=True
                self.humans[i].pref_vel_x = 0
                self.humans[i].pref_vel_y = 0
                self.humans[i].pref_omega = 0
            else:
                x_vel_world = (self.plans[i].path[1].pose.position.x - self.humans[i].x) / (self.dt)  
                y_vel_world = (self.plans[i].path[1].pose.position.y - self.humans[i].y) / (self.dt) 

                orientation_list = [self.plans[i].path[1].pose.orientation.x, self.plans[i].path[1].pose.orientation.y, 
                                    self.plans[i].path[1].pose.orientation.z, self.plans[i].path[1].pose.orientation.w]
                (_, _, yaw_world) = euler_from_quaternion (orientation_list)

                omega_world = normalize_theta(yaw_world - self.humans[i].theta)/ (self.dt)
                self.humans[i].pref_vel_x = x_vel_world
                self.humans[i].pref_vel_y = y_vel_world
                self.humans[i].pref_omega = omega_world
                del self.plans[i].path[1]
    
    def update_goals(self, goals):
        for goal in goals:
           self.humans[goal.id].goal = goal.goal
           self.humans[goal.id].goal_reached = False
           
            
            
if __name__=='__main__':
    #Controller Settings
    control_freq = 20
    planning_freq = 20.0
    look_ahead_poses = 5

    # ORCA Sim Parameters
    num_hum = 1 #Number of humans
    map_name = 'open_space'
    timestep = 1/60.0
    neighborDist = 1.5
    maxNeighbors = 3 
    timeHorizon = 2.
    timeHorizonObst = 4. 
    radius = 0.4 
    maxSpeed = 1.5
    dt = 1.0/control_freq
   
    #Initialize ROS node
    rospy.init_node('ORCA_node')


    #Start the ORCA instance
    sim = ORCA_Sim(map_name, num_hum, dt, look_ahead_poses, timestep, neighborDist, maxNeighbors, timeHorizon, timeHorizonObst, radius, maxSpeed)

    #Use message filters for multiple subscriber callback
    agent_sub = []
    agent_vel_pub = []
    goals = []
    for i in range(0,num_hum):
        name = 'human' + str(i+1)
        agent_sub.append(message_filters.Subscriber("/" + name + "/odom", Odometry))
        pub = rospy.Publisher("/" + name + "/cmd_vel", Twist, queue_size=10)
        agent_vel_pub.append(pub)
        goal = Goal(i, [None, None])
        goals.append(goal)
    _msg = message_filters.TimeSynchronizer(agent_sub, 10)
    _msg.registerCallback(sim.humansCB)
    rospy.wait_for_service(plan_srv_name_)
    get_plan_service = rospy.ServiceProxy(plan_srv_name_, MakeNavPlan)
    

    control_rate = rospy.Rate(control_freq)
    updated = False
    flip = False
    rospy.sleep(1.0)
    last_time = rospy.Time.now()
    while not rospy.is_shutdown():
        if not updated:
            if flip: 
                goals[0].goal = [4.2, 1.4]
                # goals[1].goal = [9, 4]
                # goals[2].goal = [8, 2]
            else:
                goals[0].goal = [4.2, 8.4]
                # goals[1].goal = [1, 4]
                # goals[2].goal = [1.5, 8]
            sim.update_goals(goals)
            sim.get_plans(get_plan_service)
            updated = True
        
        if np.linalg.norm([sim.humans[0].goal[0] - sim.humans[0].x, sim.humans[0].goal[1] - sim.humans[0].y]) < 0.2:
            flip = not flip
            updated = False

        # if (rospy.Time.now()-last_time).to_sec() > 1.0/planning_freq:
        #     sim.get_plans(get_plan_service)
        #     last_time = rospy.Time.now()
        
        sim.get_plans(get_plan_service)
        vels = sim.get_velocities()
        agent_vel_pub[0].publish(vels[0])
        # agent_vel_pub[1].publish(vels[1])
        # agent_vel_pub[2].publish(vels[2])
        control_rate.sleep()

