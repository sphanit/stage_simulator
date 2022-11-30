import rospy
from geometry_msgs.msg import Twist

def velCB(msg, data):
    data[0].append(msg.linear.x)
    data[1].append(msg.linear.y)
    data[2].append(msg.angular.z)

if __name__=='__main__':
    data = [[],[],[]]
    rospy.init_node('plot_vel')
    rospy.Subscriber('/human1/cmd_vel', Twist, velCB, data)
    r = rospy.Rate(10)
    try:
        print('starting')
        while not rospy.is_shutdown():
            r.sleep()
    except(rospy.exceptions.ROSInterruptException):
        print(data)
