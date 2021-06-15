#!/usr/bin/env python
import rospy
from std_msgs.msg import String
from gazebo_msgs.msg import ContactsState

contact = 0
def callback(data):
    global contact
    #rospy.loginfo(str(data.states))
    if(str(data.states) == '[]'):
        contact = 0
    else:
        contact = 1
    if(contact):
        rospy.loginfo('contato')
    
    
    
    
def listener():

    rospy.init_node('bumper_listener', anonymous=True)

    rospy.Subscriber("finger_middle_link_3_bumper", ContactsState, callback)
    global contact
    
    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                             

if __name__ == '__main__':
    listener()