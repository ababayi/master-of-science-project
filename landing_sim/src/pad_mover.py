#!/usr/bin/env python

''' File Description:

This is python file that will move the pad (moving target) in a square-shape trajectory.
Varabiles or Properties that you can change:
- waypoints
- xvel
- yvel

'''


import rospy
from geometry_msgs.msg import Twist # Twist = vector3 linear + vector3 angular (velocity)
from gazebo_msgs.msg import ModelState
from gazebo_msgs.msg import ModelStates



model_name = ""
model_state_pub = None
model_state_sub = None


# waypoints (waypoints are the square sides) positions 
waypoints = [[-10,10],[10,10],[10,-10],[-10,-10]]

# Speed in square corners
x_vel = 1.0
y_vel = 1.0

# Speeds in waypoints : 1 2 3 4
xvel = [0, 1.0, 0, -1.0]
yvel = [1.0, 0, -1.0 , 0]

waypoints_index = 0

def init():
    ''' Notes:
    
    Initialises velocities for pad. Sets up Subscriber and Publisher for model states  

    '''    
    global model_name, model_state_pub, x_vel, y_vel, model_state_sub
    rospy.init_node('moving_helipad')
    model_name = rospy.get_param("~model_name", "marker3")
    x_vel = rospy.get_param("~x_vel", 1.0) # getting "x_vel" parameter, if it does not have value, it will add 1.0 to it
    y_vel = rospy.get_param("~y_vel", 1.0) # similar to "x_vel"
    seconds_before_moving = rospy.get_param("~seconds_before_moving", 10)
    rospy.sleep(seconds_before_moving)

    # Getting the list of gazebo models with the states of them from the "gazebo/model_states" topic
    model_state_sub = rospy.Subscriber("gazebo/model_states", ModelStates, modelStatesCallback)

    # Make a publisher for sending data to the gazebo through the "gazebo/set_model_state" topic
    model_state_pub = rospy.Publisher("gazebo/set_model_state", ModelState, queue_size=1)
    while not rospy.is_shutdown():
        rospy.spin()


# Call-Back Fcn for received data from the Gazebo
def modelStatesCallback(msg):
    ''' Notes:

    Callback Function for reading data that received through the "gazebo/model_states" topic, then find the 
    marker3 model (through all of the gazebo model) and by comparing the "marker3 model state" and the defined
    "way-points" force the marker to have a square shaped movement (by passing twist velocity)

    '''
    
    global model_name, model_state_pub, x_vel, y_vel, model_state_sub, waypoints_index,waypoints
    index_of_interest = -1

    # Find the index nubmer of the marker3 and save it in "index_of_interest"
    for i in range(len(msg.name)):
        if msg.name[i] == model_name:
            index_of_interest = i
            break

    # Check if the marker3 find and create a gazebo state for it and add pose to it
    if index_of_interest >= 0:
        model_state = ModelState()
        model_state.model_name = model_name
        model_state.pose = msg.pose[index_of_interest] # pose = position + orientation
        twist = Twist() # linear + angular
        

    # This logical section create a square-shape movement
    if(abs(model_state.pose.position.x - waypoints[waypoints_index][0]) < 0.1 and abs(model_state.pose.position.y - waypoints[waypoints_index][1])<0.1):
        waypoints_index = (waypoints_index + 1) % 4 # baghi-mandeh 1%4 = 1
        
        # Check x-axis
        if(waypoints[waypoints_index][0] - waypoints[waypoints_index-1][0] == 0):
            twist.linear.x = 0.0 
        elif(waypoints[waypoints_index][0] > 0):
            twist.linear.x = abs(x_vel)
        else:
            twist.linear.x = -abs(x_vel)
        
        # Check y-axis
        if(waypoints[waypoints_index][1] - waypoints[waypoints_index-1][1] == 0):
            twist.linear.y = 0.0 
        elif(waypoints[waypoints_index][1] > 0):
            twist.linear.y = abs(y_vel)
        else:
            twist.linear.y = -abs(y_vel)
             
      
        # Check the marker twist velocity and if there is some change in velocity, apply it
        if msg.twist[index_of_interest] != twist:
            model_state.twist = twist
            model_state_pub.publish(model_state)
    
    else: # if the marker is not near the corneres
        
        twist.linear.x = xvel[waypoints_index]
        twist.linear.y = yvel[waypoints_index]
        model_state.twist = twist
        model_state_pub.publish(model_state)
    
    
if __name__ == '__main__':
    init()
