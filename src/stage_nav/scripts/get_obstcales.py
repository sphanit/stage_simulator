import numpy
import rospy
from geometry_msgs.msg import PointStamped

def add_point(msg, args):
    args.append([msg.point.x, msg.point.y])
    
    print(args)

if __name__ == '__main__':
    rospy.init_node('get_points')
    point_vec = list()
    point_sub = rospy.Subscriber('clicked_point',PointStamped,add_point, point_vec)
    rospy.spin()