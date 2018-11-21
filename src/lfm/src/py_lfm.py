#! usr/bin/env python
import rospy
from lfm.msg import Action, AprilTagDetectionArray, AprilTagDetection
from tf.transformations import euler_from_quaternion, quaternion_inverse, quaternion_multiply
import numpy as np

num_rows = 3
num_cols = 3
grid_size = np.array([num_rows, num_cols])

before_snapshot = rospy.get_param('blocks')

initial_link_prob = 0.5
detections = {}
# print type(detections)

link_prob = np.ones([(2 * num_rows - 1), (2 * num_cols - 1)])
for i in range(link_prob.shape[0]):
    for j in range(link_prob.shape[1]):
        if (i % 2 == 1 and j % 2 == 1):
            link_prob[i,j] = 0.0
        elif ((i + j) % 2 == 1):
            link_prob[i,j] = initial_link_prob

# def update_blocks():
    # after_snapshot = before_snapshot
    # after_snapshot
    # rospy.set_param(blocks)
    # before_snapshot = after_snapshot

# def capture_scene():


# def get_displacement():
#     after_

def transformCamToArm(cam):
    R = rospy.get_param('R')
    t = rospy.get_param('t')
    print R, t
    return R * cam + t; 

def detectionsClbk(msg):
    # print type(detections)
    detections_raw = msg.detections
    indices_raw = [detections_raw[i].id[0] for i in range(len(detections_raw))]
    # print indices_raw
    # print len(detections)
    for i in range(len(indices_raw)):
        # detections.update({i:detections_raw[i].pose.pose.pose})
        # detections[indices_raw[i]] = detections_raw[indices_raw[i]].pose.pose.pose
        detections[indices_raw[i]] = detections_raw[i].pose.pose.pose

    print detections[19]

    
def get_base_orientation():
    anchor_tag_id = rospy.get_param('blocks')[0]['id']


# class Scene:
#     def __init__(self):
#         self.

def run():
    pub = rospy.Publisher('chatter', Action, queue_size=10)
    detections_sub = rospy.Subscriber('/tag_detections', AprilTagDetectionArray, detectionsClbk)
    rospy.init_node('py_lfm', anonymous=True)
    rate = rospy.Rate(10) # 10hz
    while not rospy.is_shutdown():
        msg = Action()
        msg.target_tag = 19
        # hello_str = "hello world %s" % rospy.get_time()
        # rospy.loginfo(hello_str)
        pub.publish(msg)
        rate.sleep()

if __name__ == '__main__':
    try:
        run()
    except rospy.ROSInterruptException:
        pass