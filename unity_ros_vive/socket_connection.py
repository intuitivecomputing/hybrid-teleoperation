#!/usr/bin/env python

import rospy
from std_msgs.msg import String, Float64, Float64MultiArray
from geometry_msgs.msg import PoseStamped, TransformStamped
from sensor_msgs.msg import Joy
import tf.transformations
import socket
from math import pi
import numpy

RATE = 20

# degree to radians
def d2r(i):
    return float(i) * pi / 180.0

def fillin_pos(data, rot=[0, 0, 0, 0]):
    #rot is the offset of orientation for given frame
    pose = PoseStamped()
    pose.header.stamp = rospy.Time.now()
    pose.header.frame_id = "world"
    pose.pose.position.x = float(data[0])
    pose.pose.position.y = float(data[1])
    pose.pose.position.z = float(data[2])
    pose.pose.orientation.x = float(data[3])
    pose.pose.orientation.y = float(data[4])
    pose.pose.orientation.z = float(data[5])
    pose.pose.orientation.w = float(data[6])

    msg = TransformStamped()
    msg.header.stamp = rospy.Time.now()
    msg.header.frame_id = "world"
    msg.child_frame_id = "test"
    msg.transform.translation.x = float(data[0])
    msg.transform.translation.y = float(data[1])
    msg.transform.translation.z = float(data[2])
    msg.transform.rotation.x = float(data[3])
    msg.transform.rotation.y = float(data[4])
    msg.transform.rotation.z = float(data[5])
    msg.transform.rotation.w = float(data[6])

    return pose, msg

def fillin_joy(data, ctrl_name):
    msg = Joy()
    msg.header.stamp = rospy.Time.now()
    msg.header.frame_id = ctrl_name
    msg.axes =  [float(i) for i in data[0:3]]
    msg.buttons = [int(i) for i in data[3:8]]
    return msg


def vive_status_pub():
    # connect state and reconnect counter
    con_state = False
    con_cnt = 0

    # This one is send to control the head camera
    pub_head  = rospy.Publisher('/Head_Motion', PoseStamped, queue_size=1)
    
    # Those are send to control the end effectors
    pub_pos_r = rospy.Publisher('/Right_Hand',TransformStamped, queue_size=1)
    pub_pos_l = rospy.Publisher('/Left_Hand', TransformStamped, queue_size=1)

    pub_key_l = rospy.Publisher('/Left_Buttons', Joy, queue_size=1)
    pub_key_r = rospy.Publisher('/Right_Buttons', Joy, queue_size=1)

    rospy.init_node('vive_status', anonymous=True)
    rate = rospy.Rate(RATE)

    while not rospy.is_shutdown():
        
        try:
            sock.settimeout(1)
            buffer, addr = sock.recvfrom(2048)
            if not con_state:
                print("connected")
                con_state = True

            buffer = buffer.decode()
            buffer = buffer.split(',')


            head_pos, head_tran= fillin_pos(buffer[1:8])
            right_pos, msg_r = fillin_pos(buffer[9:16])
            left_pos, msg_l  = fillin_pos(buffer[17:24])

            joy_r = fillin_joy(buffer[25:32], 'right_buttons')
            joy_l = fillin_joy(buffer[33:40], 'left_buttons')


            pub_pos_r.publish(msg_r)
            pub_pos_l.publish(msg_l)
            pub_head.publish(head_pos)
            pub_key_r.publish(joy_r)
            pub_key_l.publish(joy_l)



            rate.sleep()
        except:
            print('no server detected, reconnecting ' + str(con_cnt))
            con_state = False
            con_cnt += 1
            rate.sleep()
            message = 'request'
            sock.sendto(message.encode(), address)

if __name__ == '__main__':
    address = ("IP", 23023)
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    message = 'request'
    sock.sendto(message.encode(), address)
    try:
        vive_status_pub()
    except rospy.ROSInterruptException:
        pass
