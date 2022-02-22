import rospy
from geometry_msgs.msg import Twist, Point32
from sensor_msgs.msg import LaserScan, PointCloud
from tf.transformations import euler_from_quaternion
import tf
import math


PUBLISHING_RATE = 0.1 # The rate at which we publish navigation commands
#MODE_PARAMETER = "/simulation" # The boolean parameter we get from the launch file
cmd_topic = '/cmd_vel_mux/input/navi'# The topic on which we publish velocity commands
rviz_topic = '/rviz_point_cloud' # The topic on which we publish the point cloud
laser_topic = '/scan' # The topic on which we receive the laser data


# This class is dedicated to autonomous navigation and point publishing on rviz
class AutonomousNav():

    def __init__(self):
        
        self.commands = Twist() # Navigation commands

        self.point_2D = PointCloud() # PointCloud for obstacles

        self.isTurning = False # Flag to detect that a movement is being made

        # Get the launch file parameter that specifies if we are in simulation
        #self.mode = rospy.get_param(MODE_PARAMETER) 

        # Publisher for nav commands
        self.commandPublisher = rospy.Publisher(
            cmd_topic,
            Twist, queue_size=10)

        #if(self.mode): # If in simulation
        self.FORWARD_SPEED_MPS = 2
        
        # Assign the first velocity commands
        self.commands.linear.x = self.FORWARD_SPEED_MPS
        self.commands.angular.z = 0.0

        # Publisher for 2D Points
        self.rvizPublisher = rospy.Publisher(
            rviz_topic,
            PointCloud, queue_size=10
        )
        # Publish the point at rate PUBLISHING_RATE
        rospy.Timer(rospy.Duration(PUBLISHING_RATE), self.display_points, oneshot=False)

        
        # In simulation, "laser_callback_sim" is called
        rospy.Subscriber(laser_topic, LaserScan, self.laser_callback_sim)
        rospy.Timer(rospy.Duration(PUBLISHING_RATE),
                        self.move_command, oneshot=False)
       
    # Publish commands at rate PUBLISHING_RATE while in simulation
    def move_command(self, data):
        if(not self.isTurning):
            self.commandPublisher.publish(self.commands)

    # Publish commands at rate PUBLISHING_RATE while in simulation
    def display_points(self, data):
        self.point_2D.header.frame_id = 'base_link'
        self.rvizPublisher.publish(self.point_2D)

    # Callback for laser data while in simulation
    def laser_callback_sim(self, data: LaserScan):
        isTurning = False
        self.point_2D.points.clear()
        obstacles = []
        angle = data.angle_min
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
        front = data.ranges[0]

        # Left value in all the laser rays
        left = data.ranges[15]

        # Right value in all the laser rays
        right = data.ranges[345]

        thr1 = 0.6  # Laser scan range threshold
        thr2 = 1  # Turning threshold

        # Checks if there are obstacles in front and 15 degrees left and right
        if front > thr1 and left > thr2 and right > thr2 and not isTurning:
            # If not, go forward (linear velocity)
            self.commands.linear.x = self.FORWARD_SPEED_MPS
            self.commands.angular.z = 0.0  # do not rotate (angular velocity)
        else:
            if(front < thr1): # obstacle in front
                self.commands.linear.x = 0.0
                if(left < right): # more room at right
                    self.commands.angular.z = data.angle_max # turn right
                    isTurning = True
                else: # more room at left
                    self.commands.angular.z = - data.angle_max # turn left
            if (left < thr2): # obstacle at left
                self.commands.linear.x = 0.0  # stop
                self.commands.angular.z = data.angle_max  # rotate counter-clockwise
                isTurning = True
            elif (right < thr2): # obstacle at right
                self.commands.linear.x = 0.0  # stop
                self.commands.angular.z = - data.angle_max  # rotate clockwise
                isTurning = True
            if front > thr1 and (left > thr2) and (right > thr2): # back to no danger
                isTurning = False
                self.commands.linear.x = self.FORWARD_SPEED_MPS
                self.commands.angular.z = 0.0

    # Callback function for IRL laser data
    def laser_callback(self, data: LaserScan):
        self.point_2D.points.clear()
        obstacles = []
        angle = data.angle_min
        # Initialize angles of the object detected on both sides
        # The aim is to detect the maximum turn to perform in order to 
        # get the detected obstacle out of sight 
        angle_left = 0 
        angle_right = 0
        
        thr = 0.3  # Laser scan range threshold to consider an object an obstacle
        
        # Go over all the laser rays
        for index, aDistance in enumerate(data.ranges):
            if 0.1 < aDistance and aDistance < 2.0:
                aPoint = Point32()
                aPoint.x = math.cos(angle) * aDistance
                aPoint.y = math.sin(angle) * aDistance
                aPoint.z = 0.0
                obstacles.append(aPoint)
            angle += data.angle_increment

            # If object is a close obstacle but is not a wrong value
            if(aDistance < thr and aDistance >= data.range_min):
                # For values at right
                if index in range(200, math.floor(len(data.ranges)/2)):
                    if ((index * 0.36 - 200 * 0.36) * math.pi / 180) > angle_right:
                        self.isTurning = True
                        angle_right = (index * 0.36 - 200 *
                                       0.36) * math.pi / 180
                # For values at left
                elif index in range(math.floor(len(data.ranges)/2), 580):
                    if (abs((index * 0.36 - 580 * 0.36) * math.pi / 180)) > abs(angle_left):
                        self.isTurning = True
                        angle_left = (index * 0.36 - 580 *
                                      0.36) * math.pi / 180
                else: # No danger, increment slowly the speed up to max speed
                    if(self.commands.linear.x < self.FORWARD_SPEED_MPS) :
                        self.commands.linear.x += self.FORWARD_SPEED_MPS / 4
                    else : # use max speed
                        self.commands.linear.x = self.FORWARD_SPEED_MPS
                    self.commands.angular.z = 0.0 # stop turning
                    self.isTurning = False
            else: # no need to turn
                self.isTurning = False
                # Increment up to max speed
                if(self.commands.linear.x < self.FORWARD_SPEED_MPS) :
                    self.commands.linear.x += self.FORWARD_SPEED_MPS / 4
                else :
                    self.commands.linear.x = self.FORWARD_SPEED_MPS
                self.commands.angular.z = 0.0
        # Handle on which side to actually turn
        if (angle_left != 0 and angle_right != 0):
        # if we are in a corner (obstacles on both sides)
            self.isTurning = True
            self.commands.angular.z = data.angle_max + 0.1 # turn right always
            self.commands.linear.x  = self.FORWARD_SPEED_MPS/4
        if abs(angle_left) > abs(angle_right):
            # Bigger obstacle at left
            self.isTurning = True
            self.commands.angular.z = angle_left + 0.1 # Turn left
            self.commands.linear.x = self.FORWARD_SPEED_MPS / 4
        elif angle_right != 0.0: # No obstacle left, so check if there is one on the right side
            self.isTurning = True
            self.commands.angular.z = angle_right + 0.1 # Turn right
            self.commands.linear.x = self.FORWARD_SPEED_MPS / 4
        self.commandPublisher.publish(self.commands)

        # Publish the Point Cloud to rviz
        for point in obstacles:
            self.point_2D.points.append(point)


if __name__ == '__main__':
    rospy.init_node('Move and Avoid Obstacles', anonymous=True)
    # Wait for user key press, useful to setup screen recording before robot is moving
    node = AutonomousNav()
    rospy.spin()
