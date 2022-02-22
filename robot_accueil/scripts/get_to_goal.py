#!/usr/bin/python3
import rospy , tf , tf.transformations , math , numpy
from geometry_msgs.msg import PoseStamped , Twist
import cv2 as cv 
from nav_msgs.msg import Odometry 
from sensor_msgs.msg import LaserScan 
from sensor_msgs.msg import PointCloud2


FORWARD_SPEED_MPS = 1
ANGULAR_TURN = 1.5
TURN_SPEED_MPS = 1

class Move_to:
    def __init__(self):
        self.commands = Twist()
        self.point_2D = PointCloud2() 
        self.tfListener = tf.TransformListener()
        self.goal= PoseStamped()
        self.goal.header.frame_id = '/map'
        self.robot_move_to_goal =  False
        #rospy.Subscriber('scan', LaserScan, self.callback_laser)
        self.goal_listener = rospy.Subscriber('/move_base_simple/goal' , PoseStamped, self.callback_goal )
        rospy.Subscriber("odom", Odometry , self.robot_position)
        self.pose_robot = PoseStamped()
        self.map_point = PoseStamped()
        self.command_pub = rospy.Publisher(
            '/cmd_vel_mux/input/navi',
            Twist, queue_size=10)
        rospy.Timer(rospy.Duration(0.1),self.move_robot, oneshot = False)
    
    def callback_laser(self,data) : 
        laserData = data
        nb_values = len(laserData.ranges)
        # splitting the right and left sides of the robot
        right = laserData.ranges[:math.floor(nb_values/6)]
        left = laserData.ranges[math.floor(5 * nb_values/6):]

        obstacles = []
        angle = data.angle_min
        for aDistance in data.ranges :
            if 0.1 < aDistance and aDistance < 2.0 :
                aPoint= [ 
                    math.cos(angle) * aDistance, 
                    math.sin( angle ) * aDistance
                ]
                obstacles.append( aPoint )
            angle+= data.angle_increment
        
        for point in obstacles : 
            # if point is in a dangerous zone
            if ( point[0] > 0.05 and point[0] < 0.5 and abs(point[1]) < 0.3): 
                self.point_2D.data = point
                #if the point is really close, speed up and reverse
                if(point[0] < 0.2):
                    self.commands.linear.x = - 2 * TURN_SPEED_MPS
                else : 
                    self.commands.linear.x = TURN_SPEED_MPS 
                #if there is more room on the right side, go right
                if ((numpy.amax(right) > numpy.amax(left) and self.commands.linear.x > 0 )
                or (numpy.amax(right) < numpy.amax(left) and self.commands.linear.x < 0 )):      
                    self.commands.angular.z = - ANGULAR_TURN
                # else, go left
                else :
                    self.commands.angular.z = ANGULAR_TURN
            # if not dangerous, stay as it is
            else :
                self.commands.linear.x = FORWARD_SPEED_MPS
                self.commands.angular.z = 0.0


    def callback_goal(self,data):
        self.robot_move_to_goal=True
        data.header.stamp = rospy.Time(0)
        self.goal = self.tfListener.transformPose('/map', data )
        return self.goal
 
    def robot_position(self ,data):
        self.pose_robot.pose = data.pose.pose
        self.pose_robot.header.frame_id = data.header.frame_id
        #self.map_point = self.tfListener.transformPose('/map', self.pose_robotn )
        
    
    def move_robot(self , data ):
        self.tfListener.waitForTransform("/map", "/base_footprint", rospy.Time.now(), rospy.Duration(0.1))
        #self.pose_robot.header.stamp =  rospy.Time(0)
        #self.goal.header.stamp = rospy.Time(0)
        self.map_point = self.tfListener.transformPose('/base_footprint', self.pose_robot )
        local_goal = self.tfListener.transformPose('/base_footprint', self.goal)
        if self.robot_move_to_goal :
            distance =  math.sqrt((local_goal.pose.position.x -self.map_point.pose.position.x)**2+(local_goal.pose.position.y-self.map_point.pose.position.y)**2)
            angle =  math.atan2(local_goal.pose.position.y - self.map_point.pose.position.y , local_goal.pose.position.x - self.map_point.pose.position.x)         
            if distance > 0.001 : 
                self.commands.angular.z =  (angle-self.map_point.pose.orientation.z) * 4
                self.commands.linear.x =  distance * 0.5      
            else : 
                print('goal achieved')          
            # if angle > 0.3 : 
            #     self.commands.angular.z = angle
            #     if distance > 0.1 :  
            #     #self.commands.angular.z = angle
            #         self.commands.linear.x =  distance * 0.05    
            #     else : 
            #         self.commands.linear.x =  0.0
            #         # self.commands.linear.y =  0.0
            #         # self.commands.linear.z =  0.0
            #         # self.commands.angular.x =  0.0
            #         # self.commands.angular.y =  0.0
            # else : 
            #     self.robot_move_to_goal = False
            #     self.commands.angular.z =  0.0
                # self.commands.linear.x =  distance * 0.5    
                # self.commands.angular.z = abs((angle-self.map_point.pose.orientation.z)) * 0.01  
            print(distance , angle,self.commands.linear.x , self.commands.angular.z )
            self.move_command(self.commands)


    def move_command(self, data):
        self.command_pub.publish(data)

if __name__ == '__main__':
    rospy.init_node('Goal_node', anonymous=True) 
    node = Move_to()
    rospy.spin()