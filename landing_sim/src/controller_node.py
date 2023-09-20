#!/usr/bin/env python

''' File Description:

This is python file that will contoll the quad-copter to land on the moving target.

Varabiles or Properties that you can change:
- wmax
- fmin
- fmax

'''

from __future__ import print_function, division
import quadrocoptertrajectory as quadtraj
from tf_velocity_estimator.msg import PosesAndVelocities
from tf_velocity_estimator.msg import Velocity
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import TwistStamped
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import rospy
from quadrocoptertrajectory import SingleAxisTrajectory
from std_msgs.msg import Bool
import subprocess
from plot import pidTuner, timeVsDist, trajectory3D
import numpy as np
from gazebo_msgs.msg import ModelState

# Initiate The Node
rospy.init_node('controller_minsnap_node')

# Define the trajectory starting state
pos0 = [0, 0, 2] # position @ start
vel0 = [0, 0, 0] # velocity @ start
acc0 = [0, 9.81, 0] # acceleration @ start (Assump fixed in process)

# Define the goal state
posf = [1, 0, 1]  # position @ end
velf = [0, 0, 1]  # velocity @ end
accf = [0, 9.81, 0]  # acceleration @ end (Assump fixed in process)

# Define the duration (Tfinal): deafult
Tf = 1

# Define the input limits:
fmin = 5  #[m/s**2]
fmax = 30 #[m/s**2] 25

wmax = 4 #[rad/s] 20
minTimeSec = 0.02 #[s]

# Define how gravity lies:
gravity = [0,0,-9.81]

# Get the current time
t_init = rospy.get_time()

# Create RapidTrajectory obj
traj = quadtraj.RapidTrajectory(pos0, vel0, acc0, gravity)

# Some other params for loop
feasible = False
counter  = 0

# Define the landind parameters 
landing_executed  = False
landing_threshold = 0.05 # 0.05
motor_turn_off_z = 0.07


timestampsList = []
distanceFromGround = [] # z of quad
xline, yline = [], []




def gen_traj(pos0, vel0, acc0, posf, velf, accf):
    
    '''
    Based on initial and final position, velocity and acceleration to generate a time stamped trajectory
    '''
    
    global traj
    
    traj._axis = [SingleAxisTrajectory(pos0[i],vel0[i],acc0[i]) for i in range(3)]
    traj.set_goal_position(posf)
    traj.set_goal_velocity(velf)
    traj.set_goal_acceleration(accf)
    
    # Run the algorithm, and generate the trajectory.
    traj.generate(Tf)
    print("Trajectory Generated\n---------------")
    print('pos0 and vel0: ', pos0, vel0) #/)
    print('posf and velf: ', posf, velf) #/)

    inputsFeasible = traj.check_input_feasibility(fmin, fmax, wmax, minTimeSec)
    
    # Test whether we fly into the floor
    floorPoint = [0,0,0]  # a point on the floor
    floorNormal = [0,0,1]  # we want to be in this direction of the point (upwards)
    positionFeasible = traj.check_position_feasibility(floorPoint, floorNormal)
    if(inputsFeasible == 0 and positionFeasible == 0):
    # IF Feasiable, return trajectory.
        feasible = True
    else:
        feasible = False


def callback_pad(msg):
    
    '''
    
    Callback to read pad velocity and set the final/goal position, velocity and acceleration

    Input:
    *   /gazebo/set_model_state topic
    Output:
    *   posf
    *   velf
    
    '''
    
    global pos0, vel0, acc0, posf, velf, accf, t_init, counter, landing_executed

    # Define the average var's for the position and velosity and calculate them in the next step
    avg_vx, avg_vy, asvg_vz =0,0,0

    avg_vx = msg.twist.linear.x
    avg_vy = msg.twist.linear.y
    avg_vz = msg.twist.linear.z
    
    x = msg.pose.position.x
    y = msg.pose.position.y
    z = landing_threshold
    
    posf = [x, y, z]
    velf = [avg_vx, avg_vy, avg_vz]


    counter = counter + 1 
    if( counter>100 and landing_executed == False ):
        counter= 0
        t_init = rospy.get_time()
        gen_traj(pos0, vel0, acc0, posf, velf, accf)
        #print('posf and velf: ', posf, velf)
    
    
def callback_quad(msg):
    '''
    
    Callback from setting inital position and velocity from ground truth quadrotor states
   
    Input:
    *   /ground_truth/state topic
    Output:
    *   pos0
    *   vel0
     
    '''
    global pos0,vel0,posf, landing_threshold, landing_executed
    x=msg.pose.pose.position.x
    y=msg.pose.pose.position.y
    z=msg.pose.pose.position.z    
    pos0 = [x,y,z]
    vx =msg.twist.twist.linear.x
    vy =msg.twist.twist.linear.y
    vz =msg.twist.twist.linear.z
    vel0 = [vx, vy, vz]

        
# Subscribing
sub_quadstate =  rospy.Subscriber('/ground_truth/state', Odometry, callback_quad, queue_size = 10)

#sub_padstate  = rospy.Subscriber('/pad_velocity', PosesAndVelocities, callback_pad, queue_size = 10)
sub_padstate = rospy.Subscriber("gazebo/set_model_state", ModelState, callback_pad)

# Publishing on the quadcopter
pub_pos = rospy.Publisher('/command/pose',  PoseStamped, queue_size = 10)
pub_vel = rospy.Publisher('/command/twist', TwistStamped, queue_size = 10)
pub_vel_msg = rospy.Publisher('/cmd_vel', Twist, queue_size = 10)

pos_goal_msg = PoseStamped()
vel_goal_msg = Twist()

msg = rospy.wait_for_message("gazebo/set_model_state", PosesAndVelocities , timeout=5)

t_init = rospy.get_time() # Current time = the time that trajectory initiate
gen_traj(pos0, vel0, acc0, posf, velf, accf)

# Create a publisher for the pid tuner (msg type = True/False)
flag_publisher = rospy.Publisher('/pid_tuner',  Bool, queue_size = 10)


'''
Plotting Code for PID Controller (later will be added)
'''

numPlotPoints = 100000
time = np.zeros(numPlotPoints)
position = np.zeros([numPlotPoints, 3])
velocity = np.zeros([numPlotPoints, 3])
acceleration = np.zeros([numPlotPoints, 3])
thrust = np.zeros([numPlotPoints, 1])
ratesMagn = np.zeros([numPlotPoints,1])


# charge flag
i = 0
flag=Bool()
flag.data = True
for i in range(10):
    flag_publisher.publish(flag)

delta_t=0.1


# ROS main loop
while not rospy.is_shutdown():
    flag_publisher.publish(flag)

    #Generate Trajectories
    t = rospy.get_time() # Get the current time

    if(True): # Check that the trajectory complete or not (it will complete when t-t_init = Tf)
        try:
    
            # Extracting the calculated velocity
            velocity_ = traj.get_velocity(t - t_init)
            vel_goal_msg.linear.x = velocity_[0]
            vel_goal_msg.linear.y = velocity_[1]
            vel_goal_msg.linear.z = velocity_[2]
            
            # Extracting the calculated position
            position_ = traj.get_position(t- t_init + delta_t)
            pos_goal_msg.header.frame_id = 'world'
            pos_goal_msg.pose.position.x = position_[0]
            pos_goal_msg.pose.position.y = position_[1]
            pos_goal_msg.pose.position.z = position_[2]
            
            # Plotting preparation
            # Saving time, position, velocity, acceleration, thrust, body rate
            time[i] = t - t_init
            position[i, :] = traj.get_position(t - t_init)
            velocity[i, :] = traj.get_velocity(t - t_init)
            acceleration[i, :] = traj.get_acceleration(t - t_init)
            thrust[i] = traj.get_thrust(t - t_init)
            ratesMagn[i] = np.linalg.norm(traj.get_body_rates(t - t_init))
            i = i + 1
            
            timestampsList.append(t)
            distanceFromGround.append(pos0[2]) 
            xline.append(pos0[0])
            yline.append(pos0[1])

            # Publish Data
            conmsg = [thrust, ratesMagn]
            pub_vel_msg.publish(conmsg)
            
            #Shut down the quadrotor if landing was executed
            if(abs(pos_goal_msg.pose.position.z) < motor_turn_off_z): # landing_executed == True
                print("Shutting Down Quadrotor")
                vel_goal_msg.linear.x = 0
                vel_goal_msg.linear.y = 0
                vel_goal_msg.linear.z = 0
                pub_vel_msg.publish(vel_goal_msg)
                ret = subprocess.call(['rosservice call /enable_motors "enable: false"'], shell=True)
                break
                break
                
        except:
            print("Error (Out of Time) -> Use previous trajectory")
            continue
        
    else:
       if(landing_executed == True):
           break
       i=0

timeVsDist(timestampsList, distanceFromGround)
trajectory3D(xline, yline, distanceFromGround)
ret = pidTuner(time, position, velocity, acceleration, thrust, ratesMagn, Tf, fmin, fmax, wmax)
flag.data = ret
flag_publisher.publish(flag)