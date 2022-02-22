#!/usr/bin/python3
import rospy , tf , tf.transformations , math , numpy
from geometry_msgs.msg import PoseStamped , Twist , Point32
import cv2 as cv 
from nav_msgs.msg import Odometry 
from sensor_msgs.msg import LaserScan 
from sensor_msgs.msg import PointCloud

PUBLISHING_RATE = 0.1 # The rate at which we publish navigation commands
FORWARD_SPEED_MPS = 1.4



thr1 = 0.6  # Laser scan range threshold
thr2 = 1  # Turning threshold


class Move_to:
    def __init__(self):
        self.commands = Twist()
        self.tfListener = tf.TransformListener()
        self.goal= PoseStamped() #position of goal
        self.pose_robot = PoseStamped() #position of robot
        self.map_point = PoseStamped() #position of robot in basefootprint
        self.robot_move_to_goal =  False 

        self.point_2D = PointCloud() # PointCloud for obstacles
        self.isTurning = False # Flag to detect that a movement is being made


        self.goal.header.frame_id = '/map'
        rospy.Subscriber('scan', LaserScan, self.callback_laser)
        self.goal_listener = rospy.Subscriber(
            '/move_base_simple/goal' ,
            PoseStamped, self.callback_goal )
        rospy.Subscriber("odom", Odometry , self.robot_position)
        self.command_pub = rospy.Publisher(
            '/cmd_vel_mux/input/navi',
            Twist, queue_size=10)
        rospy.Timer(rospy.Duration(0.1),self.move_robot, oneshot = False)
    


    def callback_goal(self,data):
        self.robot_move_to_goal=True
        data.header.stamp = rospy.Time(0)
        self.goal = self.tfListener.transformPose('/map', data )
        return self.goal

    def callback_laser(self,data :LaserScan): 
        self.isTurning = False
        self.point_2D.points.clear()
        obstacles = []
        angle = data.angle_min
        self.angle_maxi =  data.angle_max
        # Compute the laser data to get coordinates relative to the robot
        for aDistance in data.ranges:
            if 0.1 < aDistance and aDistance < 2.0:
                aPoint = Point32()
                aPoint.x = math.cos(angle) * aDistance
                aPoint.y = math.sin(angle) * aDistance
                aPoint.z = 0.0
                obstacles.append(aPoint)
            angle += data.angle_increment
        # Assign each obstacle found to a Point Cloud
        for point in obstacles:
            self.point_2D.points.append(point)
        
        # Front value in all the laser rays
        self.front = data.ranges[0]

        # Left value in all the laser rays
        self.left = data.ranges[15]

        # Right value in all the laser rays
        self.right = data.ranges[345]

 
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
        #print(self.map_point ,self.pose_robot )
        isTurning = False
        distance =  math.sqrt((local_goal.pose.position.x -self.map_point.pose.position.x)**2+(local_goal.pose.position.y-self.map_point.pose.position.y)**2)
        angle =  math.atan2(local_goal.pose.position.y - self.map_point.pose.position.y , local_goal.pose.position.x - self.map_point.pose.position.x)         
        self.mode= ' nothg'
        
        if self.robot_move_to_goal :
            if distance > 0.001 : 
                if self.front > thr1 and (self.left > thr2) and (self.right > thr2) and not isTurning:
                    self.commands.angular.z =  (angle-self.map_point.pose.orientation.z) * 4
                    self.commands.linear.x =  distance * 0.5 
                    self.mode = 'go to goal'   
                if(self.front < thr1): # obstacle in front
                    if(self.left < self.right): # more room at right
                        self.commands.angular.z =   self.angle_maxi # turn right
                        isTurning = True
                        self.commands.linear.x = FORWARD_SPEED_MPS
                        self.mode =  'obstacle devant , tr'
                    else: # more room at left
                        self.commands.angular.z = - self.angle_maxi # turn left
                        self.commands.linear.x = FORWARD_SPEED_MPS
                        self.mode = 'obstacle devant , tl'                      
                if (self.left < thr2): # obstacle at left
                    self.commands.linear.x = 0.0  # stop
                    self.commands.angular.z =  self.angle_maxi # rotate counter-clockwise
                    isTurning = True
                    self.mode = 'obstacle left , tr'
                elif (self.right < thr2): # obstacle at right
                    self.commands.linear.x = 0.0  # stop
                    self.commands.angular.z = - self.angle_maxi  # rotate clockwise  
                    isTurning = True
                    self.mode = 'obstacle right , tl'
                if self.front > thr1 and (self.left > thr2) and (self.right > thr2): # back to no danger
                    isTurning = False
                    self.commands.angular.z =  (angle-self.map_point.pose.orientation.z) * 4
                    self.commands.linear.x =  distance * 0.5 
                    self.mode = 'plus d obstacle '
            else : 
                print('goal achieved')   
                self.robot_move_to_goal= False    
        
        self.move_command(self.commands)
        print(self.robot_move_to_goal , self.mode , self.left ,self.right , self.angle_maxi)



    def move_command(self, data):
        self.command_pub.publish(data)
        
if __name__ == '__main__':
    rospy.init_node('Goal_node', anonymous=True) 
    node = Move_to()
    rospy.spin()