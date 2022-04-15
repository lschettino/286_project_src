import rospy
import actionlib
from move_base_msgs.msg import MoveBaseAction , MoveBaseGoal
from tf.transformations import quaternion_from_euler, euler_from_quaternion
from std_srvs.srv import Empty
from std_msgs.msg import Float32
import math
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Point, Quaternion
from wsr_toolbox_cpp.msg import wsr_aoa_array
import argparse
import numpy as np

msg = """
control your Turtlebot3!
-----------------------
Insert xyz - coordinate.
x : position x (m)
y : position y (m)
z : orientation z (degree: -180 ~ 180)
If you want to close, insert 's'
-----------------------
"""
PI = 3.14

class RobotSLAM_Nav:
    def __init__(self, goal_topic, position_topic,bot,beacon):
        rospy.init_node("move_base_tester")
        self.client = actionlib.SimpleActionClient(goal_topic,MoveBaseAction)
        self.timeout = 10 #secs
        self.step_size = 0.5
        self.beacon = beacon # only taking 1 beacon 

        if(bot == 1):
            #Clear the costmap and rtabmap using rosservice calls.
            rospy.wait_for_service('/locobot/rtabmap/reset')
            reset_map = rospy.ServiceProxy('/locobot/rtabmap/reset', Empty)
            reset_map()

            rospy.wait_for_service('/locobot/move_base/clear_costmaps')
            clear_costmap = rospy.ServiceProxy('/locobot/move_base/clear_costmaps', Empty)
            clear_costmap()
        else:
            #Clear the costmap and rtabmap using rosservice calls.
            rospy.wait_for_service('/rtabmap/reset')
            reset_map = rospy.ServiceProxy('/rtabmap/reset', Empty)
            reset_map()

            rospy.wait_for_service('/move_base/clear_costmaps')
            clear_costmap = rospy.ServiceProxy('/move_base/clear_costmaps', Empty)
            clear_costmap()


        #Create the actionlib server
        self.client.wait_for_server()

        #Initialize the variable for the goal
        self.goal = MoveBaseGoal()
        self.goal.target_pose.header.frame_id = "map"

	self.gotAOA = False

	# Initialize position based on odometer measurements (will be 0,0 on hardware and (x_init,y_init) in simulated)
	init_odom = rospy.wait_for_message(position_topic, Odometry)
        self.current_position = init_odom.pose.pose.position # Type Point
        self.current_ori = init_odom.pose.pose.orientation  # Type Quaternion
	
	# Orientation in degrees in the xy plane with respect to the initial orientation (1,0 vector)
	self.current_yaw_angle = euler_from_quaternion([self.current_ori.x,self.current_ori.y,self.current_ori.z,self.current_ori.w])[2] * (180/PI)
	print("INITIAL ORIENTATION ", self.current_yaw_angle)

	# Define Topics for which to subscribe
        rospy.Subscriber(position_topic, Odometry, self.odom_cb)
        rospy.Subscriber('aoa_topic', Float32, self.aoa_cb_dummy)
        
    def odom_cb(self,msg):
	#Update this callback function to get the positions of the robot from its odometery.
	#Use the variables self.current_position and self.current_ori to store the position and orientation values.
	self.current_position.x = msg.pose.pose.position.x
	self.current_position.y = msg.pose.pose.position.y 
        rot = Quaternion(msg.pose.pose.orientation.x, msg.pose.pose.orientation.y, msg.pose.pose.orientation.z, msg.pose.pose.orientation.w) 
	self.current_ori = rot 
	self.current_yaw_angle = euler_from_quaternion([rot.x,rot.y,rot.z,rot.w])[2] * (180/PI)

    

    def aoa_cb_dummy(self, msg):
        self.move_direction = msg.data 
        self.gotAOA = True
        print("Got new AOA")

    def print_current_state(self, header):
	distance = round(np.sqrt((self.beacon.x - self.current_position.x) ** 2 + (self.beacon.y - self.current_position.y) ** 2),3)
	print(header + " Robot Location: (" + str(self.current_position.x)+','+str(self.current_position.y) +"), Beacon Location:  (" + str(self.beacon.x)+','+str(self.beacon.y) +"), Current distance: "+str(distance)+ " AOA: " + str(self.move_direction))


    
    def get_AOA(self):
	AOA = self.beacon.simulate_AOA(self.current_position.x, self.current_position.y, self.current_yaw_angle, noise=False)  
	
	self.move_direction = AOA 

	self.gotAOA = True 
    
    # Getter method to return the xy coordinates of the robot
    def get_coords(self):
	return (self.current_position.x,self.current_position.y)
	

    def move_along_direction(self, max_iter=100):
        rospy.loginfo("Waiting for AOA...\n")
 	distance = _distance(self.get_coords(), self.beacon.get_coords() )

	#counter of number of iterations
	iter_cnt = 0
	

	while not rospy.is_shutdown():

	    iter_cnt += 1 
	    
	    if iter_cnt <= max_iter:
		self.get_AOA()
 		distance = _distance(self.get_coords(), self.beacon.get_coords() )
	    

	    # Condition for moving: 1) robot received new AOA, 2) robot is far from target 3)max number of iterations

            if(self.gotAOA and distance > 0.3 and iter_cnt <= max_iter):

	        self.print_current_state("\n\nIteration "+ str(iter_cnt) + " POS1 " )
		angle = self.move_direction / 180 * math.pi # (to convert to radians) 

		delta_x = self.step_size * math.cos(angle) 
		delta_y = self.step_size * math.sin(angle) 
		# Add your code here to update x and y based on the AOA direction and step-size

		x = self.current_position.x + delta_x 
                y = self.current_position.y + delta_y
			
                print("Moving to next location x = ",x, ", y =",y)
                
                self.goal.target_pose.header.stamp = rospy.Time.now()
                self.goal.target_pose.pose.position.x = x
                self.goal.target_pose.pose.position.y = y
		

		goal_orientation = quaternion_from_euler(0,0, self.move_direction) # roll, pitch, yaw

		print(angle)
		print(self.move_direction)
		print(goal_orientation)
		self.goal.target_pose.pose.orientation = Quaternion(*goal_orientation)
                #self.goal.target_pose.pose.orientation.x = self.current_ori.x
                #self.goal.target_pose.pose.orientation.y = self.current_ori.y
                #self.goal.target_pose.pose.orientation.z = self.current_ori.z
                #self.goal.target_pose.pose.orientation.w = self.current_ori.w
                
                rospy.loginfo("Attempting to move to the goal")
                self.client.send_goal(self.goal)
                wait=self.client.wait_for_result(rospy.Duration(self.timeout))

                if not wait:
                    rospy.loginfo("Timed-out after failing to reach the goal.")

                    rospy.loginfo("Please provide a new goal position")
                else:
                    rospy.loginfo("Reached goal successfully")
		
		# Check correctness on this
		self.client.cancel_goal()
                self.gotAOA = False



		
	
class WifiBeacon:
    def __init__(self, name, x, y):
	"""
	x and y are relative to the robots initial position (0,0)
	"""
	self.name = name
	self.x = x
	self.y = y
	

	# angle (polar coordinate angle) with respect to robot's (0,0,00) heading (x_pos, y_pos, orientation)
	# x = r * cos(theta), y = r * sin(theta)
	# r = sqrt(x^2+y^2) 
	# so theta = arctan(y/x) but arctan doesn't work for for negative coordinates
	

    def simulate_AOA(self, robot_x, robot_y, robot_orientation, noise=False):
	'''
	Method to return

	'''
	# Coords of beacon - coords of robot
	delta_x = self.x - robot_x
	delta_y = self.y - robot_y

	angle = _cartesian_to_polar(delta_x,delta_y) - robot_orientation
	
	if noise:
	    angle += np.random.normal(0,2)
	
	return angle

    # Getter function for xy coordinates
    def get_coords(self):
	return (self.x,self.y)
	


def _cartesian_to_polar(x,y):
    r = math.sqrt(x*x+y*y)
	
    cos_theta = x/r
	
    # compute angle in radians
    angle = math.acos(cos_theta) * (180/PI)
    if y < 0:
        angle *= -1
	
    # return angle is from 180 to -180
    return angle

def _distance(a,b):
    return np.sqrt((a[0] - b[0]) ** 2 + (a[1] - b[1])**2)




	
def simulation_0():
    # maximum number of iterations that the robot can take to find the beacon
    max_iterations = 10
    beacon_1 = WifiBeacon('beacon1', 5, -5)
    obj = RobotSLAM_Nav('/move_base', '/odom', 2, beacon_1)
    
    obj.move_along_direction(max_iterations) 







if __name__=='__main__':
   # parser = argparse.ArgumentParser(description='Get the inputs.')
   # parser.add_argument('--goal_topic', type=str)
   # parser.add_argument('--position_topic', type=str)
   # parser.add_argument('--bot', type=int)
   # args = parser.parse_args()
   # obj = RobotSLAM_Nav(args.goal_topic, args.position_topic, args.bot)
   simulation_0()
    
    #obj.move()
    #obj.move_along_direction()






