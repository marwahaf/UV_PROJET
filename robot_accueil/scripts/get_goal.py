#!/usr/bin/python3
import rospy , tf , tf.transformations , math , numpy
from geometry_msgs.msg import PoseStamped , Twist , Point32, Pose
import cv2 as cv 
from nav_msgs.msg import Odometry 
from sensor_msgs.msg import LaserScan 
from sensor_msgs.msg import PointCloud


PUBLISHING_RATE = 0.1 # The rate at which we publish navigation commands
FORWARD_SPEED_MPS = 1
TURN_SPEED_MPS = 1


thr1 = 0.7 # Laser scan range threshold
thr2 = 1 # Turning threshold 


class Move_to:
    def __init__(self):
        self.point_right = []
        self.point_left = [] 
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
            # '/mobile_base/commands/velocity',
            '/cmd_vel_mux/input/navi',
            Twist, queue_size=10)
        rospy.Timer(rospy.Duration(0.1),self.move_robot, oneshot = False)
        
        self.point_publish = rospy.Publisher('/pointcloud', PointCloud , queue_size = 10)
        
        # Publisher for 2D Points
        self.rvizPublisher = rospy.Publisher( '/rviz_point_cloud', PointCloud, queue_size=10)
        # Publish the point at rate PUBLISHING_RATE
        rospy.Timer(rospy.Duration(0.05), self.display_points, oneshot=False)

        
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
        self.increment= data.angle_increment
        self.point_right.clear()
        self.point_left.clear() 
        # Compute the laser data to get coordinates relative to the robot
        for aDistance in data.ranges:
            if 0.01 < aDistance and aDistance < 0.7 : #mettre 0.7 au lieu de 2 en IRL   
                aPoint = Point32()
                aPoint.x = math.cos(angle) * aDistance
                aPoint.y = math.sin(angle) * aDistance
                aPoint.z = 0.0
                obstacles.append(aPoint)
            angle += data.angle_increment
        
        # Assign each obstacle found to a Point Cloud
        for point in obstacles:
            self.point_2D.points.append(point)
        #print(self.point_2D)
            
        # n = (- data.angle_min)/self.increment
        # d = (-math.pi/3 - data.angle_min)/self.increment
        # g = (math.pi/3 - data.angle_min)/self.increment

        # #print(2*math.pi/3, int(n) ,int(d) , int(g), data.angle_max , data.angle_min , self.increment)
       
        # # Front value in all the laser rays
        # self.front = data.ranges[int(n)]

        # # Left value in all the laser rays
        # self.left = data.ranges[int(g)]

        # # Right value in all the laser rays
        # self.right = data.ranges[int(d)]
 
    def robot_position(self ,data):
        self.pose_robot.pose = data.pose.pose
        self.pose_robot.header.frame_id = data.header.frame_id
        #self.map_point = self.tfListener.transformPose('/map', self.pose_robotn )
        
    
    def move_robot(self , data ):
        isTurning_right = False
        #isTurning = False

        self.tfListener.waitForTransform("/map", "/base_footprint", rospy.Time.now(), rospy.Duration(0.1))
        self.map_point = self.tfListener.transformPose('/base_footprint', self.pose_robot )
        local_goal = self.tfListener.transformPose('/base_footprint', self.goal)   
       
        distance =  math.sqrt((local_goal.pose.position.x -self.map_point.pose.position.x)**2+(local_goal.pose.position.y-self.map_point.pose.position.y)**2)
        angle =  math.atan2(local_goal.pose.position.y - self.map_point.pose.position.y , local_goal.pose.position.x - self.map_point.pose.position.x)         
        
        self.mode= ' nothg'
        
        if self.robot_move_to_goal :
            if distance > 0.1 : 
                if self.point_2D.points == []:
                    self.commands.angular.z =  (angle-self.map_point.pose.orientation.z) * 4
                    self.commands.linear.x =  min(distance * 0.5 , FORWARD_SPEED_MPS)
                    self.mode = 'go to goal'   
                else : 
                    for point in self.point_2D.points :
                        if point.x > 0 :
                            self.point_right += [point]
                            print( self.point_right)
                            # point_proche_x_r = min(self.point_right.x)
                        else: 
                            self.point_left += [point] 
                            # point_proche_x_l = max(point.x) 
                        if point.y < 0.5 :
                            # self.commands.linear.x = 0.0  # stop
                            if point.x < 0.3 and point.x > 0 :
                                #self.commands.linear.x = 0.0  # stop
                                self.commands.angular.z =   0.7 * self.angle_maxi   # rotate  clockwise  
                                self.commands.linear.x = FORWARD_SPEED_MPS
                                isTurning_right = True
                                self.mode = 'obstacle right , turn l'
                            elif abs(point.x) < 0.3 and point.x < 0 and isTurning_right == False:
                                #self.commands.linear.x = 0.0  # stop
                                self.commands.angular.z = - 0.7 * self.angle_maxi  # rotate counterclockwise
                                self.commands.linear.x = FORWARD_SPEED_MPS
                                self.mode = 'obstacle left , turn r'
                            elif abs(point.x) < 0.3 and point.x < 0 and isTurning_right == True:
                                self.commands.linear.x = - FORWARD_SPEED_MPS
                                if point_proche_x_r < abs(point_proche_x_l):
                                    self.commands.angular.z =  0.7 * self.angle_maxi 
                                    isTurning_right = True
                                    self.mode = 'obstacle des 2 côtés , obs_droite '
                                else :
                                    self.commands.angular.z = - 0.7 * self.angle_maxi  # rotate clockwise
                                    self.mode = 'obstacle des 2 côtés , obs_gauche '
                        else  : 
                            self.commands.angular.z =  (angle-self.map_point.pose.orientation.z) * 4
                            self.commands.linear.x =  min(distance * 0.5 , FORWARD_SPEED_MPS)
                            self.mode = 'go to goal'   
                            isTurning_right = False
                    print(point.x , point.y)
                # elif(self.front < thr1): # obstacle in front
                #     if (self.left < thr2) and (self.right < thr2):#corner
                #         self.commands.angular.z = self.angle_maxi
                #         self.commands.linear.x = - 1.0
                #         isTurning = True
                #         isTurning_right =  False
                #         self.mode = 'coin?'
                #     elif (self.right < thr2): #obstacle at right+front 
                #         isTurning_right =  False
                #         isTurning = True
                #         self.commands.angular.z = - min(self.angle_maxi , TURN_SPEED_MPS) # turn left
                #         self.commands.linear.x = FORWARD_SPEED_MPS
                #         self.mode = 'obstacle devant +right, tl' 
                #     elif (self.left < thr2):   #obstacle at left+front 
                #         isTurning = True
                #         isTurning_right =  True
                #         self.commands.angular.z = min(self.angle_maxi , TURN_SPEED_MPS) # turn right   
                #         self.commands.linear.x = FORWARD_SPEED_MPS        
                #         self.mode =  'obstacle devant +left, tr'
                #     else : 
                #         if(self.left < self.right): # more room at right
                #             isTurning = True
                #             isTurning_right =  True
                #             self.commands.angular.z = min(self.angle_maxi , TURN_SPEED_MPS) # turn right   
                #             self.commands.linear.x = 1.0         
                #             self.mode =  'obstacle devant slmt , tr'
                #         elif isTurning_right == False: # more room at left
                #             isTurning = True
                #             self.commands.angular.z = - min(self.angle_maxi , TURN_SPEED_MPS) # turn left
                #             self.commands.linear.x = 1.0
                #             self.mode = 'obstacle devant slmt, tl'      
                # # elif (self.left < thr2): # obstacle at left
                # #     self.commands.linear.x = 0.0  # stop
                # #     self.commands.angular.z = min(self.angle_maxi , TURN_SPEED_MPS) # rotate counter-clockwise
                # #     isTurning = True
                # #     self.mode = 'obstacle left , tr'
                # #     if(self.front > thr1) and (self.left > thr2):
                # #         self.commands.linear.x = 0.5 
                # #         self.mode = 'obstacle left , tr + a'    
                # #     else : 
                # #        self.commands.angular.z = min(self.angle_maxi , TURN_SPEED_MPS) 
                # #        self.mode = 'obstacle left , tr + t'
                # # elif (self.right < thr2): # obstacle at right
                # #     self.commands.linear.x = 0.0  # stop
                # #     self.commands.angular.z = - min(self.angle_maxi , TURN_SPEED_MPS)  # rotate clockwise  
                # #     isTurning = True
                # #     self.mode = 'obstacle right , tl'
                # #     if(self.front > thr1) and (self.right > thr2):
                # #         self.commands.linear.x = 0.5 
                # #         self.mode = 'obstacle right , tl + a'
                # #     else : 
                # #        self.commands.angular.z = min(self.angle_maxi , TURN_SPEED_MPS) 
                # #        self.mode = 'obstacle right , tl + t'
                # # # elif (self.left < thr2) and (self.right < thr2) and (self.front < thr1) : 
                # # #     self.commands.linear.x = - FORWARD_SPEED_MPS
                # # #     self.commands.angular.z = 0.0
                # # #     isTurning = False
                # # #     isTurning_right =  False
                # # #     self.mode = 'coin?'
                # # elif (self.front > thr1) and (self.left > thr2) and (self.right > thr2): # back to no danger
                # #     isTurning = False
                # #     isTurning_right =  False
                # #     self.commands.angular.z =  (angle-self.map_point.pose.orientation.z) * 4
                # #     self.commands.linear.x =  min( distance * 0.5 , FORWARD_SPEED_MPS) 
                # #     self.mode = 'plus d obstacle '
                # # # else: 
                # # #     self.commands.angular.z =  0.0
                # # #     self.commands.linear.x =  - FORWARD_SPEED_MPS
                # # #     self.mode = 'obstacle partout'
                
            else : 
                self.mode = 'goal achieved'  
                self.robot_move_to_goal= False    
                self.commands.angular.z = 0.0
                self.commands.linear.x = 0.0
        
        self.move_command(self.commands)
        print(self.robot_move_to_goal , self.mode  )
        #print(self.robot_move_to_goal , self.mode , distance , self.commands.angular.z ,  self.commands.linear.x )
        # print(self.robot_move_to_goal , self.mode , self.front , self.right, self.left , self.front < thr1 , self.left < thr2 ,self.right<thr2)
    
    # Publish commands at rate PUBLISHING_RATE while in simulation
    def display_points(self, data):
        self.point_2D.header.frame_id = 'base_link'
        self.rvizPublisher.publish(self.point_2D)


    def move_command(self, data):
        self.command_pub.publish(data)
        
if __name__ == '__main__':
    rospy.init_node('Goal_node', anonymous=True) 
    node = Move_to()
    rospy.spin()