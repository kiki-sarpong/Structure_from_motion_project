from sensor_msgs.msg import Image
import cv2
import numpy as np
import rospy
import sys
from cv_bridge import CvBridge, CvBridgeError

class read_cam(object):
    def __init__(self):
<<<<<<< HEAD
        self.top_left_image = None
        self.top_right_image = None
        self.left_image = None
        self.right_image = None
        self.back_left_image = None
        self.back_right_image = None
        self.br = CvBridge()
        self.path = "/home/user/simulation_ws/src/camera_package/scripts/calib6"
=======
        self.forward_image = None
        self.backward_image = None
        self.left_image = None
        self.right_image = None
        self.br = CvBridge()
        self.path = "/home/user/simulation_ws/src/camera_package/scripts/images"
>>>>>>> a26fc77222fede9eb319640cdd1147ada6b93615
        self.cnt = 0 

        #publishers
        self.pub = rospy.Publisher("/camera/read_image",Image,queue_size=10)

        #subscribers
<<<<<<< HEAD
        self.sub_top_left = rospy.Subscriber("/work_env_cam/camera1/image_raw",Image,self.callback_top_left)   
        self.sub_top_right = rospy.Subscriber("/work_env_cam/camera2/image_raw",Image,self.callback_top_right)    
        self.sub_right = rospy.Subscriber("/work_env_cam/camera3/image_raw",Image,self.callback_right)
        self.sub_back_right = rospy.Subscriber("/work_env_cam/camera4/image_raw",Image,self.callback_back_right) 
        self.sub_back_left = rospy.Subscriber("/work_env_cam/camera5/image_raw",Image,self.callback_back_left)   
        self.sub_left = rospy.Subscriber("/work_env_cam/camera6/image_raw",Image,self.callback_left) 


    def callback_top_left(self,img_msg):
        # rospy.loginfo("reading video.....")
        try:
            self.top_left_image = self.br.imgmsg_to_cv2(img_msg, "bgr8")
        except CvBridgeError, e:
            rospy.logerr("CvBridge Error top_camera: {0}".format(e))

    
    def callback_top_right(self,img_msg):
        # rospy.loginfo("reading video.....")
        try:
            self.top_right_image = self.br.imgmsg_to_cv2(img_msg, "bgr8")
=======
        self.sub_forward = rospy.Subscriber("/eatery_cam/camera1/image_raw",Image,self.callback_forward)   
        self.sub_backward = rospy.Subscriber("/eatery_cam/camera2/image_raw",Image,self.callback_backward) 
        self.sub_left = rospy.Subscriber("/eatery_cam/camera4/image_raw",Image,self.callback_left)   
        self.sub_right = rospy.Subscriber("/eatery_cam/camera3/image_raw",Image,self.callback_right)
    
    def callback_forward(self,img_msg):
        # rospy.loginfo("reading video.....")
        try:
            self.forward_image = self.br.imgmsg_to_cv2(img_msg, "bgr8")
        except CvBridgeError, e:
            rospy.logerr("CvBridge Error forward_camera: {0}".format(e))

    
    def callback_backward(self,img_msg):
        # rospy.loginfo("reading video.....")
        try:
            self.backward_image = self.br.imgmsg_to_cv2(img_msg, "bgr8")
>>>>>>> a26fc77222fede9eb319640cdd1147ada6b93615
        except CvBridgeError, e:
            rospy.logerr("CvBridge Error backward_camera: {0}".format(e))

    def callback_left(self,img_msg):
        # rospy.loginfo("reading video.....")
        try:
            self.left_image = self.br.imgmsg_to_cv2(img_msg, "bgr8")
        except CvBridgeError, e:
            rospy.logerr("CvBridge Error left_camera: {0}".format(e))


    def callback_right(self,img_msg):
        # rospy.loginfo("reading video.....")
        try:
            self.right_image = self.br.imgmsg_to_cv2(img_msg, "bgr8")
        except CvBridgeError, e:
            rospy.logerr("CvBridge Error right_camera: {0}".format(e))

<<<<<<< HEAD
    def callback_back_right(self,img_msg):
        # rospy.loginfo("reading video.....")
        try:
            self.back_right_image = self.br.imgmsg_to_cv2(img_msg, "bgr8")
        except CvBridgeError, e:
            rospy.logerr("CvBridge Error back_right_camera: {0}".format(e))

    def callback_back_left(self,img_msg):
        # rospy.loginfo("reading video.....")
        try:
            self.back_left_image = self.br.imgmsg_to_cv2(img_msg, "bgr8")
        except CvBridgeError, e:
            rospy.logerr("CvBridge Error back_left_camera: {0}".format(e))

=======
>>>>>>> a26fc77222fede9eb319640cdd1147ada6b93615
    def image_analyis(self):
        # rospy.loginfo("Timing images")
        #rospy.spin()
        while not rospy.is_shutdown():
<<<<<<< HEAD
            active = np.array([(self.top_left_image > 0),(self.top_right_image > 0) , (self.left_image > 0), (self.right_image > 0), (self.back_left_image > 0), (self.back_right_image> 0)])
            cond = active.shape    # get the shape... if an image isn't prest len(cond) = 1
            if len(cond) == 4:

                if cv2.waitKey(1) & 0XFF == ord("s"):
                    name = self.path + "/" + "image_" + str(self.cnt) + ".jpg"
                    cv2.imwrite(name,self.left_image)
=======
            if self.forward_image is not None: 
                if cv2.waitKey(1) & 0XFF == ord("s"):
                    name = self.path + "/" + "image_" + str(self.cnt) + ".jpg"
                    cv2.imwrite(name,self.forward_image)
>>>>>>> a26fc77222fede9eb319640cdd1147ada6b93615
                    rospy.loginfo(name)
                    self.cnt +=1
               

<<<<<<< HEAD
                final_img = self.left_image          #final image to be shown
=======
                final_img = self.forward_image          #final image to be shown
>>>>>>> a26fc77222fede9eb319640cdd1147ada6b93615
                try:
                    self.pub.publish(self.br.cv2_to_imgmsg(final_img))
                except CvBridgeError, e:
                    rospy.logerr("CvBridge Error: {0}".format(e))

                cv2.imshow("camera_feed",final_img)
                if cv2.waitKey(1) & 0XFF ==ord("q"):
                    break
            # self.rate.sleep()
    
def main(args):
    rospy.init_node('cam_node', anonymous=True)
    eatery_cam = read_cam()
    eatery_cam.image_analyis()
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")
    cv2.destroyAllWindows()


if __name__ == '__main__':
    main(sys.argv)