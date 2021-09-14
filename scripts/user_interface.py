"""
.. module:: user_interface
    :platform: Unix
    :synopsis: Python module to implement a user_interface in order to command the activation of a robot

.. moduleauthor:: Ilenia D'Angelo

ROS node to activate a robot in order to implement the reaching of random goals

Subscribes to:
None

Publishes to:
None

Service :
/user_interface

"""

import rospy
import time
from rt2_assignment1.srv import Command

## Documentation for the main function.
#
#  More details.
#
# @var ui_client defines a service client of user_interface type. It taskes as argument the Command service to activate/deactivate robot behaviours according to user's preferences  
# @var x it stores the input from the user

def main():
    rospy.init_node('user_interface')
    ui_client = rospy.ServiceProxy('/user_interface', Command)
    time.sleep(10)
    rate = rospy.Rate(20)
    # x stores the input of the user that is casted as int
    x = int(input("\nPress 1 to start the robot "))
    while not rospy.is_shutdown():
        if (x == 1):
            # Sending the string "start" as request of the service /user_interface
            ui_client("start") 
            x = int(input("\nPress 0 to stop the robot "))
        else:
            print("The robot is stopped")
            # Sending the string "stop" as request of the service /user_interface
            ui_client("stop")
            x = int(input("\nPress 1 to start the robot "))
            
if __name__ == '__main__':
    main()
