#!/usr/bin/env python
import rospy
from std_msgs.msg import String
from nav_msgs.msg import Odometry
import tf
from tf.transformations import euler_from_quaternion
import numpy as np
from geometry_msgs.msg import Twist
import time
from gazebo_msgs.msg import ModelStates
from std_msgs.msg       import Float64
payload_state = None
d, s = 1, 1
global i, j, k

def payload(msg):
    global payload_state
    global i, j, k
    idx = msg.name.index('payload')
    payload_state = msg.pose[idx]
    orientation_q = payload_state.orientation
    orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
    (roll, pitch, yaw) = euler_from_quaternion (orientation_list)

    print(np.rad2deg(roll), np.rad2deg(pitch))

    # # rolln = (10*np.tan(-roll))
    # # pitchn = (10*np.tan(pitch))
    # rolln = (10*np.tan(-roll))
    # pitchn = (10*np.tan(-pitch)) 
    # #A = np.array([[-d*0.5, -s*.5],[-d*.5, s*.5],[d*.5, -s*.5],[d*.5, s*.5]])
    # A = np.array([[-d*0.5, s*.5],[-d*.5, -s*.5],[d*.5, s*.5],[d*.5, -s*.5]])
    # state = np.array([[rolln], [pitchn]])
    # control = np.dot(A,state)
 
    # initialcontrol = np.array([[0.12], [0.12], [0.12], [0.12]])  #L0 = 10cm
    # controln = control + initialcontrol   # L = Delta(L) + L0
    # print(round(controln[0], 3))
    # print(round(controln[1], 3))
    # print(round(controln[2], 3))
    # print(round(controln[3], 3))
    # print('----------------')
    # # #try1 Working rolln = roll & pitchn = pitch
    # # l0_pub.publish(round(controln[2], 3))
    # # l1_pub.publish(round(controln[3], 3))
    # # l2_pub.publish(round(controln[0], 3))
    # # l3_pub.publish(round(controln[1], 3))
    # l0_pub.publish(round(controln[0], 3))
    # l1_pub.publish(round(controln[1], 3))
    # l2_pub.publish(round(controln[2], 3))
    # l3_pub.publish(round(controln[3], 3))

    theta_x = 10 * np.tan(-roll)
    theta_y = 10 * np.tan(pitch)
    x0, y0 = -0.5, 1
    x1, y1 = -0.5, -1
    x2, y2 = 1, 0.5 
  
    A = np.array([[y0, x0],[y1, x1],[y2, x2]])
    state = np.array([[theta_x], [theta_y]])
    control = np.dot(A,state)
    initialcontrol = np.array([[0.12], [0.12], [0.12]])
    controln = control + initialcontrol
    print(round(controln[0], 3))
    print(round(controln[1], 3))
    print(round(controln[2], 3))

    print('----------------')
    l0_pub.publish(round(controln[0], 3))
    l1_pub.publish(round(controln[1], 3))
    l2_pub.publish(round(controln[2], 3))


    # Ainv = np.linalg.pinv(A)
    # controlnn = np.array([controln[0,0], controln[1,0],controln[2,0],controln[3,0]])
    # statenn = np.dot(Ainv,controlnn)
    # print(np.rad2deg(statenn[0])*0.1, np.rad2deg(statenn[1])*0.1)


if __name__ == '__main__':
    rospy.init_node('pub_sub', anonymous=True)
    rospy.Subscriber('/gazebo/model_states', ModelStates, payload)
    rate = rospy.Rate(10) # 10hz
    
    vel_pub0 = rospy.Publisher('/minion_0/diff_drive_controller/cmd_vel', Twist, queue_size=10)
    vel_pub1 = rospy.Publisher('/minion_1/diff_drive_controller/cmd_vel', Twist, queue_size=10)
    vel_pub2 = rospy.Publisher('/minion_2/diff_drive_controller/cmd_vel', Twist, queue_size=10)
    vel_pub3 = rospy.Publisher('/minion_3/diff_drive_controller/cmd_vel', Twist, queue_size=10)

    l0_pub = rospy.Publisher('/minion_0/effort_controllers/command', Float64, queue_size=10)
    l1_pub = rospy.Publisher('/minion_1/effort_controllers/command', Float64, queue_size=10)
    l2_pub = rospy.Publisher('/minion_2/effort_controllers/command', Float64, queue_size=10)
    l3_pub = rospy.Publisher('/minion_3/effort_controllers/command', Float64, queue_size=10)

    l0_pub.publish(0.12)
    l1_pub.publish(0.12)
    l2_pub.publish(0.12)
    while not rospy.is_shutdown():
        twist0 = Twist()
        twist0.linear.x = 0.08
        twist0.angular.z = 0
        vel_pub0.publish(twist0)
        twist1 = Twist()
        twist1.linear.x = 0.08
        twist1.angular.z = 0
        vel_pub1.publish(twist1)
        twist2 = Twist()
        twist2.linear.x = 0.08
        twist2.angular.z = 0
        vel_pub2.publish(twist2)
        twist3 = Twist()
        twist3.linear.x = 0.0
        twist3.angular.z = 0.0
        vel_pub3.publish(twist3)
        rate.sleep()
    rospy.spin()