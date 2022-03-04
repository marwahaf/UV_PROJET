#!/usr/bin/python3
from turtle import Turtle, TurtleScreen
import rospy , tf , tf.transformations , math , numpy
from geometry_msgs.msg import PoseStamped , Twist , Point32
import cv2 as cv 
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan 
from sensor_msgs.msg import PointCloud

PUBLISHING_RATE = 0.1           # The rate at which we publish navigation commands
SPEED_GO_GOAL = 0.5             # The linear speed of the robot when he goes to goal 
SPEED_AVOID_OBSTACLE = 0      # The linear speed of the robot when an obstacle is detected
TURNING_SPEED = 1               # The angular speed when the robot turns when obstacle detected
DIST_TOLERANCE_FORWARD = 0.5    # The distance tolerance forward the robot to be considered as an obstacle
DIST_TOLERANCE_ASIDE = 0.3      # The distance tolerance just left and right the robot to be considered as an obstacle
DIST_LASER = 1                  # The laser range potential obstacles detection
GOAL_RADIUS = 0.1

thr1 = 0.6  # Laser scan range threshold
thr2 = 0.6 # Turning threshold 


class Move_to:
    def __init__(self):
        self.commands = Twist()
        self.tfListener = tf.TransformListener()
        self.goal= PoseStamped() #position of goal
        self.pose_robot = PoseStamped() #position of robot
        self.map_point = PoseStamped() #position of robot in basefootprint
        self.robot_move_to_goal =  False 
        self.returninghome = False
        self.point_2D = PointCloud() # PointCloud for obstacles
        self.isTurning_right = False # Flag to detect that a movement is being made
        self.isTurning_left = False
        self.moving_mode = 'no move'
        self.factor = 1

        self.goal.header.frame_id = '/map'
        self.laser = rospy.Subscriber('front_scan', LaserScan, self.callback_laser)
        self.goal_listener = rospy.Subscriber(
            '/move_base_simple/goal' ,
            PoseStamped, self.callback_goal )
        self.person_listener = rospy.Subscriber(
            '/goal/person',
            PoseStamped,
            self.callback_person
        )
        rospy.Subscriber("/goal/returnhome", PoseStamped, self.returnhome)
        rospy.Subscriber("odom", Odometry , self.robot_position)
        self.command_pub = rospy.Publisher(
            '/mobile_base/commands/velocity',
            # '/cmd_vel_mux/input/navi',
            Twist, queue_size=10)
        self.arrived = rospy.Publisher('/goal/home_returned', PoseStamped,queue_size=1)
        # rospy.Timer(rospy.Duration(0.1),self.move_robot, oneshot = False)
        self.point_publish = rospy.Publisher('/pointcloud', PointCloud , queue_size = 10)


    def returnhome(self,data):
        self.returninghome = True
        self.goal.pose.position.x = 0
        self.goal.pose.position.y = 0
        self.robot_move_to_goal = True

    def callback_goal(self,data):
        self.laser.unregister()
        self.laser = rospy.Subscriber('front_scan', LaserScan, self.callback_laser)
        self.robot_move_to_goal=True
        self.moving_mode = 'To goal'
        self.factor = 1
        data.header.stamp = rospy.Time(0)
        self.goal = self.tfListener.transformPose('/map', data )

    def callback_person(self,data):
        self.laser.unregister()
        self.laser = rospy.Subscriber('back_scan', LaserScan, self.callback_laser)
        self.robot_move_to_goal = True
        self.moving_mode = 'To people'
        self.factor = -1
        data.header.stamp = rospy.Time(0)
        print("Person detected !")
        print(data)
        self.goal = self.tfListener.transformPose('/map',data)
        print(self.goal)

    def callback_laser(self,data): 
        self.isTurning = False
        self.point_2D.points.clear()
        obstacles = []
        contiguous_obstacles = []
        angle = data.angle_min
        self.angle_maxi =  data.angle_max
        # Compute the laser data to get coordinates relative to the robot
        for aDistance in data.ranges:
            if data.range_min*2 < aDistance and aDistance < DIST_LASER :   
                aPoint = Point32()
                aPoint.x = math.cos(angle) * aDistance
                aPoint.y = math.sin(angle) * aDistance
                aPoint.z = 0.0
                contiguous_obstacles.append(aPoint)
            else:
                if(contiguous_obstacles != []):
                    obstacles.append(contiguous_obstacles)
                contiguous_obstacles = []
            angle += data.angle_increment

        self.point_2D.points = obstacles
        # Front value in all the laser rays
        self.front = data.ranges[int(len(data.ranges)/2)]
        
        # Left value in all the laser rays
        self.left = data.ranges[int(len(data.ranges)/2) + int((math.pi/2)/data.angle_increment)]
        
        # Right value in all the laser rays
        self.right = data.ranges[int(len(data.ranges)/2) - int((math.pi/2)/data.angle_increment)]
        self.move_robot()
    
    def robot_position(self ,data):
        self.pose_robot.pose = data.pose.pose
        self.pose_robot.header.frame_id = data.header.frame_id
        #self.map_point = self.tfListener.transformPose('/map', self.pose_robotn )
        
    
    def move_robot(self):
        # self.tfListener.waitForTransform("/map", "/base_footprint", rospy.Time.now(), rospy.Duration(0.1))
        self.map_point = self.tfListener.transformPose('/base_footprint', self.pose_robot )
        local_goal = self.tfListener.transformPose('/base_footprint', self.goal)   
        print(local_goal.pose.position)
        distance =  math.sqrt((local_goal.pose.position.x -self.map_point.pose.position.x)**2+(local_goal.pose.position.y-self.map_point.pose.position.y)**2)
        angle =  math.atan2(local_goal.pose.position.y - self.map_point.pose.position.y , local_goal.pose.position.x - self.map_point.pose.position.x)         
        self.mode= ' nothg'

        # if goal get
        if self.robot_move_to_goal :
            # if robot is far the goal
            if distance > GOAL_RADIUS : 
                obstacles_x = []
                obstacles_y = []
                for obst in self.point_2D.points:
                    obstacles_x += [value.x for value in obst]
                    obstacles_y += [value.y for value in obst]
                point = PointCloud()
                point.header.stamp = rospy.Time(0)
                if self.factor == 1:
                    point.header.frame_id = 'laserfront_link'
                else:
                    point.header.frame_id = 'laserback_link'
                objects_in_box = []
                #Get only the points in the "protection area"
                for l in self.point_2D.points:
                    if self.factor == 1:
                        dist_tolerance = DIST_TOLERANCE_FORWARD
                    else:
                        dist_tolerance = 0.2
                    obj = [val for val in l if (val.x<dist_tolerance and abs(val.y)<DIST_TOLERANCE_ASIDE)]
                    point.points+=obj
                    if(obj != []):
                        objects_in_box.append(obj)
                # if obstacle has an x < DIST_TOLERANCE_X and an y < DIST_TOLERANCE_ASIDE => Obstacle is in a box defined forward the robot
                # if (any([val<DIST_TOLERANCE_FORWARD for val in obstacles_x]) and any([abs(val) < DIST_TOLERANCE_ASIDE for val in obstacles_y])):                      
                if objects_in_box != []:
                    self.point_publish.publish(point)
                    point.points = objects_in_box
                    #checking if the goal is also in this box => in this case, goal is considered achieved
                    if(abs(local_goal.pose.position.x)<= dist_tolerance and abs(local_goal.pose.position.y) <=  DIST_TOLERANCE_ASIDE):
                        if(self.factor == 1):
                            if local_goal.pose.position.x>=0:
                                self.mode = 'Goal & obstacle in box'
                                self.commands.angular.z = 0
                                self.commands.linear.x = 0
                                self.robot_move_to_goal = False
                                self.isTurning_left = False
                                self.isTurning_right = False
                                print('forced goal achieved')
                            else:
                                self.mode = 'obst & goal in box, goal behind'
                                
                                self.commands.angular.z = 0
                                self.commands.linear.x = - SPEED_GO_GOAL
                                self.isTurning_left = False
                                self.isTurning_right = False
                                print('Goal behind, moving backward')
                        else:
                            if local_goal.pose.position.x <= 0:
                                self.mode = 'Moving Backward, Goal & obst in box'
                                self.commands.angular.z = 0
                                self.commands.linear.x = 0
                                self.robot_move_to_goal = False
                                self.isTurning_left = False
                                self.isTurning_right = False
                                print('forced goal achieved')
                            else:
                                self.mode = 'Move backward, obst in box but not goal'
                    #if the goal is not in the "protection box"
                    else:
                        #Checking if obstacles are left or right
                        if point.points[0][0].y > 0: #first point detected is the leftest, so if he is at y>0, obstacle is left.
                            #obstacle is left, so go right
                            if not(self.isTurning_left):
                                self.isTurning_right = True
                                if self.factor ==1:
                                    self.commands.angular.z = - TURNING_SPEED
                                self.commands.linear.x = self.factor * SPEED_AVOID_OBSTACLE
                                self.mode = 'Obs FL, TR'
                        #if first point is at right
                        else: 
                            #last point detected is the rightest, so if he is at y<0, obstacle is right
                            if point.points[-1][-1].y<0: 
                                #obstacle is right, so go left
                                if not(self.isTurning_right):
                                    self.isTurning_left = True
                                    self.commands.angular.z = TURNING_SPEED
                                    self.commands.linear.x = self.factor * SPEED_AVOID_OBSTACLE
                                    self.mode = 'Obs FR, TL'
                            #if last point is not at right
                            else:
                                #check if we have one obstacle or more
                                if(len(point.points) == 1):
                                    pointsleft = [val for val in point.points[0] if val.y>=0] #get the obj points at left
                                    pointsright = [val for val in point.points[0] if val.y<0] #get the obj points at right
                                    #if more points at left than at right:
                                    if(len(pointsleft)>=len(pointsright)): 
                                        #then go right
                                        if not(self.isTurning_left):
                                            self.isTurning_right = True
                                            self.commands.angular.z =  - TURNING_SPEED
                                            self.commands.linear.x = self.factor * SPEED_AVOID_OBSTACLE
                                            self.mode = '1 Obs F, TR'
                                    #if more points at right than at left
                                    else:
                                        #then go left
                                        if not(self.isTurning_right):
                                            self.isTurning_left
                                            self.commands.angular.z = TURNING_SPEED
                                            self.commands.linear.x = self.factor * SPEED_AVOID_OBSTACLE
                                            self.mode = '1 Obs F, TL'
                                #if we have more than one obstacle
                                else:
                                    pointsleft = []
                                    pointsright = []
                                    for obsts in point.points:
                                        pointsleft += [val for val in obsts if val.x >=0]
                                        pointsright += [val for val in obsts if val.y <0]

                                    if (len(pointsleft)>= len(pointsright)):
                                        if not(self.isTurning_left):
                                            self.isTurning_right = True
                                            self.commands.angular.z = -TURNING_SPEED
                                            self.commands.linear.x = self.factor * SPEED_AVOID_OBSTACLE
                                            self.mode = '>1 obs F, TR'
                                    else:
                                        if(not(self.isTurning_right)):
                                            self.isTurning_left = True
                                            self.commands.angular.z = TURNING_SPEED
                                            self.commands.linear.x = self.factor * SPEED_AVOID_OBSTACLE
                                            self.mode = '>1 obs F, TL'
                elif (self.left <= DIST_TOLERANCE_ASIDE+0.2 or self.right <= DIST_TOLERANCE_ASIDE+0.2):
                    point.points = []
                    for l in self.point_2D.points:
                        point.points += [val for val in l]
                    self.point_publish.publish(point)
                    self.commands.angular.z = 0
                    self.commands.linear.x = self.factor * SPEED_GO_GOAL
                    self.mode = 'Obs L|R'
                    self.isTurning_left = False
                    self.isTurning_right = False
                # if no obstacles in the "protection box" of the robot
                else: 
                # just go to the goal.
                    self.point_publish.publish(point)
                    self.isTurning_left = False
                    self.isTurning_right = False
                    if self.factor == 1:
                        self.commands.angular.z =  (angle-self.map_point.pose.orientation.z) #* 4
                    else:
                        self.commands.angular.z =  math.pi + (angle-self.map_point.pose.orientation.z) #* 4
                    self.commands.linear.x =  self.factor * min(distance, SPEED_GO_GOAL)
                    self.mode = 'go to goal'
            else : 
                print('goal achieved')   
                self.isTurning_right = False
                self.isTurning_left = False
                self.robot_move_to_goal= False    
                self.commands.angular.z =  0.0
                self.commands.linear.x = 0.0
                if self.returninghome:
                    self.returninghome=False
                    self.arrived.publish(self.goal)
        
        self.move_command(self.commands)
        print(self.robot_move_to_goal ,self.moving_mode, self.mode , distance, angle)



    def move_command(self, data):
        self.command_pub.publish(data)
        
if __name__ == '__main__':
    rospy.init_node('Goal_node', anonymous=True) 
    node = Move_to()
    rospy.spin()