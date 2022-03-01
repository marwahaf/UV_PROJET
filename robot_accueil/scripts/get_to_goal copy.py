#!/usr/bin/python3
import rospy , tf , tf.transformations , math , numpy
from geometry_msgs.msg import PoseStamped , Twist
import cv2 as cv 
from nav_msgs.msg import Odometry 
from sensor_msgs.msg import LaserScan 
from sensor_msgs.msg import PointCloud2

FORWARD_SPEED_MPS = 0.5
ANGULAR_TURN = 1
TURN_SPEED_MPS = 0.1
DIST_TOLERANCE = 0.5
DIST_LASER_ROBOT = 0.5
DIST_TOLERANCE_COTE = 0.5
ANGLE_TOLERANCE = math.radians(1)


class Move_to:
    def __init__(self):
        self.commands = Twist()
        self.point_2D = PointCloud2() 
        self.tfListener = tf.TransformListener()
        self.goal = PoseStamped()
        self.local_goal= PoseStamped()
        self.robot_move_to_goal =  False
        rospy.Subscriber('scan', LaserScan, self.callback_laser)
        self.goal_listener = rospy.Subscriber('/move_base_simple/goal' , PoseStamped, self.callback_goal )
        rospy.Subscriber("odom", Odometry , self.get_pose)
        self.pose_robot = PoseStamped()
        self.map_point = PoseStamped()
        self.obj_detected = False
        self.command_pub = rospy.Publisher(
            # '/cmd_vel_mux/input/navi',
            '/mobile_base/commands/velocity',
            Twist, queue_size=1)
        #rospy.Timer(rospy.Duration(0.1),self.move_robot, oneshot = False)
        self.turnright = False
        self.turnleft = False
        self.countturn = 0
    
    def callback_goal(self,data):
        #self.local_goal est published dans la map
        self.robot_move_to_goal=True
        print(data)
        self.goal = data


    def get_pose(self ,data):
        self.pose_robot = PoseStamped(
            data.header,
            data.pose.pose
        )
        self.pose_robot.pose.position.x = round(self.pose_robot.pose.position.x,4)
        self.pose_robot.pose.position.y = round(self.pose_robot.pose.position.y,4)
        self.pose_robot.header.stamp = rospy.Time(0)
        self.map_point = self.tfListener.transformPose('/map', self.pose_robot)
        # print("Pose :", end = '')
        # print(self.map_point)
        self.move_robot()
        # self.pose_robot.pose.position.x = data.pose.pose.position.x
        # self.pose_robot.pose.position.y = data.pose.pose.position.y
        # self.pose_robot.pose.position.z = data.pose.pose.position.z
        # self.pose_robot.pose.orientation.x = data.pose.pose.orientation.x
        # self.pose_robot.pose.orientation.y = data.pose.pose.orientation.y
        # self.pose_robot.pose.orientation.z = data.pose.pose.orientation.z
        # self.pose_robot.pose.orientation.w = data.pose.pose.orientation.w
        #print(data)
        # self.pose_robot.header.frame_id = "map"
        
        
        '''(self.pos, self.rot ) = self.tfListener.lookupTransform('/odom', '/map', rospy.Time(0))
        print(self.pos, self.rot)
        distance =  abs(math.sqrt(((self.local_goal.pose.position.x -self.pos[0])**2)+(self.local_goal.pose.position.y-self.pos[1])**2))
        angle =  math.atan2(self.local_goal.pose.position.y - self.pos[1], self.local_goal.pose.position.x - self.pos[0])      
        #print( distance , angle , abs((angle-self.rot[2])))
        '''
    
    def move_robot(self):
        if self.robot_move_to_goal and not(self.obj_detected):
            self.goal.header.stamp = rospy.Time(0)
            local_goal = self.tfListener.transformPose("/base_footprint",self.goal)
            #print(local_goal)
            #distance =  abs(math.sqrt(((self.local_goal.pose.position.x -self.map_point.pose.position.x)**2)+(self.local_goal.pose.position.y-self.map_point.pose.position.y)**2))
            distance =  abs(math.sqrt(((local_goal.pose.position.x)**2)+(local_goal.pose.position.y)**2))
            angle =  math.atan2(local_goal.pose.position.y, local_goal.pose.position.x)
            #angle =  math.atan2(self.local_goal.pose.position.y - self.map_point.pose.position.y , self.local_goal.pose.position.x - self.map_point.pose.position.x) #- math.atan2(self.map_point.pose.position.y,self.map_point.pose.position.x)) 
            quat = [
                    self.map_point.pose.orientation.x,
                    self.map_point.pose.orientation.y,
                    self.map_point.pose.orientation.z,
                    self.map_point.pose.orientation.w
                ]
            theta = tf.transformations.euler_from_quaternion(quat)[2]
            if(distance >= DIST_TOLERANCE):
                self.commands.linear.x = min(distance,FORWARD_SPEED_MPS)
                if abs(angle) > ANGLE_TOLERANCE:
                    self.commands.angular.z = angle #- theta
                else:
                    self.commands.angular.z = 0
            # if (abs(angle)>0.05):
            #     if(angle > 0):
            #         self.commands.angular.z = ANGULAR_TURN
            #     else:
            #         self.commands.angular.z = - ANGULAR_TURN
            # else:
            #     self.commands.angular.z = 0
            else:
                self.robot_move_to_goal = False
                self.commands.linear.x = 0
                self.commands.angular.z = 0
            # self.commands.linear.x =  distance * 0.5    
            # self.commands.angular.z = abs((angle-self.map_point.pose.orientation.z)) * 0.01  
            print(distance, math.degrees(angle), self.commands.linear.x, math.degrees(self.commands.angular.z))
        self.move_command(self.commands)

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
                    math.cos(angle) * aDistance, #x
                    math.sin( angle ) * aDistance #y
                ]
                obstacles.append( aPoint )
            angle+= data.angle_increment

        for point in obstacles : 
            # if point is in a dangerous zone
            self.obj_detected = True
            if ( point[0] > 0.05 and point[0] < 1 and abs(point[1]) < DIST_TOLERANCE_COTE):
                print("Obj detected")
                self.point_2D.data = point
                #if the point is really close, speed up and reverse
                # if(point[0] < 0.2):
                #     # print("Very close")
                #     self.commands.linear.x = - 2 * TURN_SPEED_MPS
                # else : 
                self.commands.linear.x = TURN_SPEED_MPS 
                #if there is more room on the right side, go right
                if ((numpy.amax(right) > numpy.amax(left) and self.commands.linear.x > 0 )
                or (numpy.amax(right) < numpy.amax(left) and self.commands.linear.x < 0 )):      
                    if not(self.turnleft) or self.countturn == 0:
                        self.commands.angular.z = - ANGULAR_TURN
                # else, go left
                else :
                    if not(self.turnright) or self.countturn ==0:
                        self.commands.angular.z = ANGULAR_TURN
            # if not dangerous, stay as it is
            else :
                self.obj_detected = False
                self.turnleft = False
                self.turnright = False
                self.countturn = 0
                # print("No obj")
                # self.commands.linear.x = FORWARD_SPEED_MPS
                # self.commands.angular.z = 0.0
            # self.move_command(self.commands)

    def move_command(self, data):
        # print(data)
        self.command_pub.publish(data)

if __name__ == '__main__':
    rospy.init_node('Goal_node', anonymous=True) 
    node = Move_to()
    rospy.spin()