from sensor_msgs.msg import PointCloud,Image
from std_msgs.msg import Header, Float64, Int32, String
from geometry_msgs.msg import Point32
import roslib; roslib.load_manifest('laser_assembler')
import rospy; from laser_assembler.srv import *
import cv2
import numpy as np
import rospy
import sys
from cv_bridge import CvBridge, CvBridgeError
from scipy.sparse import lil_matrix
import time
from scipy.optimize import least_squares         #for solving least square
import tf
import rosbag
from std_msgs.msg import Int32, String

class sfm(object):
    def __init__(self):
        self.top_left_image = None
        self.top_right_image = None
        self.left_image = None
        self.right_image = None
        self.back_left_image = None
        self.back_right_image = None
        self.br = CvBridge()
        self.frame_inc = 0         #frame count increment
        self.no_RT = np.column_stack((np.eye(3,3),np.zeros(3)))

        #initiate listener transform
        self.listener = tf.TransformListener()        

        # create needed variables for all the cameras
        self.cam1_frames = []
        self.cam2_frames = []
        self.cam3_frames = []
        self.cam4_frames = []
        self.cam5_frames = []
        self.cam6_frames = []

        #cam_1 main variables
        self.cam1_first_data_done = False     #passed just for if-else loop
        self.cam1_3d_points = np.empty(None)
        self.cam1_2d_points = np.empty(None)
        self.cam1_RT = []
        self.RT_p1 = np.column_stack((np.eye(3,3),np.zeros(3)))   #set up initial rotation with identity and zero translation
        self.cam1_RT.append(self.RT_p1)      #add initial first rotation and translation 
        self.cam1_inc = 0
        self.BA_cam_RT = []
        self.BA_3d = []

        #cam_2 main variables
        self.cam2_first_data_done = False
        self.cam2_3d_points = np.empty(None)
        self.cam2_2d_points = np.empty(None)
        self.cam2_RT = []
        self.RT_p2 = np.column_stack((np.eye(3,3),np.zeros(3)))   #set up initial rotation with identity and zero translation
        self.cam2_RT.append(self.RT_p2)
        self.cam2_inc = 0
        self.BA_cam_RT = []
        self.BA_3d = []

        #cam_3 main variables
        self.cam3_first_data_done = False
        self.cam3_3d_points = np.empty(None)
        self.cam3_2d_points = np.empty(None)
        self.cam3_RT = []
        self.RT_p3 = np.column_stack((np.eye(3,3),np.zeros(3)))   #set up initial rotation with identity and zero translation
        self.cam3_RT.append(self.RT_p3)
        self.cam3_inc = 0
        self.BA_cam_RT = []
        self.BA_3d = []

        #cam_4 main variables
        self.cam4_first_data_done = False
        self.cam4_3d_points = np.empty(None)
        self.cam4_2d_points = np.empty(None)
        self.cam4_RT = []
        self.RT_p4 = np.column_stack((np.eye(3,3),np.zeros(3)))   #set up initial rotation with identity and zero translation
        self.cam4_RT.append(self.RT_p4)
        self.cam4_inc = 0
        self.BA_cam_RT = []
        self.BA_3d = []

        #cam_5 main variables
        self.cam5_first_data_done = False
        self.cam5_3d_points = np.empty(None)
        self.cam5_2d_points = np.empty(None)
        self.cam5_RT = []
        self.RT_p5 = np.column_stack((np.eye(3,3),np.zeros(3)))   #set up initial rotation with identity and zero translation
        self.cam5_RT.append(self.RT_p5)
        self.cam5_inc = 0
        self.BA_cam_RT = []
        self.BA_3d = []

        #cam_6 main variables
        self.cam6_first_data_done = False
        self.cam6_3d_points = np.empty(None)
        self.cam6_2d_points = np.empty(None)
        self.cam6_RT = []
        self.RT_p6 = np.column_stack((np.eye(3,3),np.zeros(3)))   #set up initial rotation with identity and zero translation
        self.cam6_RT.append(self.RT_p6)
        self.cam6_inc = 0
        self.BA_cam_RT = []
        self.BA_3d = []


#/**************************************************************************************************************/#
        global K1,D1, K2,D2, K3,D3, K4,D4, K5,D5, K6,D6
        #calibration from top_left_camera
        K1 = np.array([[270.2694931014879, 0.0, 540.0222458506611],
                            [0.0, 270.2217374963253, 359.9723041080024], 
                            [0.0, 0.0, 1.0]])
        D1 = np.array([[0.08324724539033564], [0.01102622729090816], [-0.009620664659349626], [0.0076961177157634526]])
        
        #calibration from top_right_camera
        K2 = np.array([[270.266093951815, 0.0, 539.5269629287891], 
                    [0.0,270.21209598183697, 359.52114120206], 
                    [0.0, 0.0, 1.0]])
        D2 = np.array([[0.08202017055956276], [0.011152284361217133],[-0.0025612851871677127], [0.0016035642038714005]])
        
        #calibration from right_camera
        K3 = np.array([[270.0526326521154, 0.0, 539.7732839810546],
                        [0.0, 269.98036134743126, 359.21132642621285],
                        [0.0, 0.0, 1.0]])
        D3 = np.array([[0.0873342666774445], [-0.004714768137375608], [0.015841884109494126], [-0.005631207575627033]])
        
        # calibration from back_right_camera
        K4 = np.array([[269.8933525073022, 0.0, 539.4722935226645],
                        [0.0, 269.86511566774413, 359.5999666092721],
                        [0.0, 0.0, 1.0]])
        D4 = np.array([[0.08146261617251875], [0.013423621797780089], [-0.00486621067021839], [0.002285829002461235]])
        
        # calibration from back_left_camera
        K5 = np.array([[269.73696932904556, 0.0, 539.5322975712345],
                        [0.0, 269.7475008959941, 359.1769875117506], 
                        [0.0, 0.0, 1.0]])
        D5 = np.array([[0.08803366140102019], [0.00016863324113027467], [0.006896101867569259], [-0.0013754227881913226]])
        
        #calibration from left_camera
        K6 = np.array([[269.62079076264683, 0.0, 540.0404538058622],
                        [0.0, 269.5071617279691, 359.5805859442406], 
                        [0.0, 0.0, 1.0]])
        D6 = np.array([[0.08366327141240004], [0.009516460957491722], [-0.0015423369572141708], [0.0012011934000748488]])
        
        
#/**************************************************************************************************************/#  
        #publishers
        self.pub = rospy.Publisher("/camera/read_image",Image,queue_size=10)
        self.cam1_pointcloud = rospy.Publisher('pointcloud', PointCloud, queue_size=50)
        self.cam2_pointcloud = rospy.Publisher('pointcloud', PointCloud, queue_size=50)
        self.cam3_pointcloud = rospy.Publisher('pointcloud', PointCloud, queue_size=50)
        self.cam4_pointcloud = rospy.Publisher('pointcloud', PointCloud, queue_size=50)
        self.cam5_pointcloud = rospy.Publisher('pointcloud', PointCloud, queue_size=50)
        self.cam6_pointcloud = rospy.Publisher('pointcloud', PointCloud, queue_size=50)
        self.final_pointcloud = rospy.Publisher('final_pointcloud', PointCloud, queue_size=50)

        #subscribers
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


#/**************************************************************************************************************/#
    def undistort(self,img,K,D):
        balance = 0.0
        dim = img.shape[:2][::-1]  #dim1 is the dimension of input image to un-distort

        # This is how scaled_K, dim2 and balance are used to determine the final K used to un-distort image. OpenCV document failed to make this clear!
        new_K = cv2.fisheye.estimateNewCameraMatrixForUndistortRectify(K, D, dim, np.eye(3), balance=balance)
        map1, map2 = cv2.fisheye.initUndistortRectifyMap(K, D, np.eye(3), new_K, dim, cv2.CV_16SC2)
        undistorted_img = cv2.remap(img, map1, map2, interpolation=cv2.INTER_LINEAR, borderMode=cv2.BORDER_CONSTANT)
        return undistorted_img


    def display_pointcloud(self):
        rospy.wait_for_service("assemble_scans")
        try:
            assemble_scans = rospy.ServiceProxy('assemble_scans', AssembleScans)
            resp = assemble_scans(rospy.Time(0,0), rospy.get_rostime())
            self.final_pointcloud.publish(resp.cloud)
            # print ("Got cloud with %u points"% len(resp.cloud.points))
        except rospy.ServiceException, e:
            print ("Service call failed: %s"%e)


    def generate_point_cloud(self,pointList, n,t,cam_frame_id,wd_frame_id):
        # offset = 2.5   #point clouds seem really close to robot model.
        sn = "/"
        cam_frame = sn + cam_frame_id    # "/....."
        wd_frame =  sn + wd_frame_id        # "/......"
        transformed_points = PointCloud()
        assert pointList.shape[1] == 3
        X = pointList[:,0]
        Y = pointList[:,1]
        Z = pointList[:,2]
        l = len(Z)
        # Define the header block
        scanHeader = Header()
        scanHeader.seq = n
        scanHeader.stamp = t
        scanHeader.frame_id = cam_frame_id
        # Generate point cloud
        points = []
        for i in range(l):
            p = Point32()
            p.x = X[i]
            p.y = Y[i]
            p.z = Z[i] 
            points.append(p)
        # Prepare the pointcloud message
        pc = PointCloud()
        pc.header = scanHeader
        pc.points = points
        try:
            # (trans,rot) = self.listener.lookupTransform(wd_frame, cam_frame, rospy.Time(0))
            # print(trans,rot)
            self.listener.waitForTransform(cam_frame, wd_frame, rospy.Time(), rospy.Duration(10.0))
            transformed_points = self.listener.transformPointCloud(wd_frame, pc)
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            pass

        return transformed_points


    def min_reprojection(self,params,n_cameras,n_points,K,point_indices,p2d_observations):   #obtains the reprojection error
        all_RT = params[:n_cameras * 12].reshape((n_cameras,3, 4))  #12 due to 12 values in R/T --convert back to 3x4 matrix
        all_3d_points = params[n_cameras * 12:].reshape((n_points, 3))
        img_points = []
        for i,rt in enumerate(all_RT):        #go over all saved rotations and translations
            proj_mat = np.dot(K,rt)           # get RT and solve for projective matrix
            pts_3d = all_3d_points[point_indices==i]
            for p3d in pts_3d:
                p2d_proj = np.dot(proj_mat,np.hstack((p3d,1)))     #add 1 to 3d point array to find the points in the x,y camera
                p2d_proj = p2d_proj/p2d_proj[-1]        # 2d proj x,y,1
                p2d_proj = np.hstack((p2d_proj[0],p2d_proj[1]))
                img_points.append(p2d_proj)      
        p2d_projection = np.array(img_points)
        reproj_error = (p2d_observations - p2d_projection)**2
        reproj_error = reproj_error.ravel()
        return reproj_error


    def create_sparsity_matrix(self,frame_count,n_points,point_indices):
    #n_points = number of points
    #point_indices = point sequences for the correspondences
    #ordered list--- frame count and observations match
        m = point_indices.size * 2
        n = frame_count * 12 + n_points * 3
        A = lil_matrix((m, n), dtype=int)
        print(A,"A matrix")

        i = np.arange(frame_count)
        j = np.arange(point_indices.size)
        for s in range(12):
            A[2 * i, i * 12 + s] = 1
            A[2 * i + 1, i * 12 + s] = 1

        for s in range(3):
            print(point_indices.shape)
            break
            A[2 * j, frame_count * 12 + point_indices * 3 + s] = 1
            A[2 * j + 1, frame_count * 12 + point_indices * 3 + s] = 1

        return A


    #get the section of data for the bundle adjustment since you're not going to be using all the data
    def get_BA_data(self,all_3d_points,all_2d_points,point_indices,inc,thresh):
        gv = np.arange(inc - thresh,inc)
        sec_3D_points = np.empty(None)
        sec_2D_points = np.empty(None)
        BA_indices = np.empty(None)          # indices for BA...this is separate from the global indices
               
        for i,ind in enumerate(gv):
            # since you want to get a local set of indices for BA
            multi_val = np.full(all_2d_points.shape[0],i)   #create arrays from 0,1,2,3,...n

            #np.append cant put together multi-dimensional arrays at once,..so I do this
            if i== 0:
                sec_2D_points = all_2d_points[point_indices==ind]
                sec_3D_points = all_3d_points[point_indices==ind]
                BA_indices = multi_val[point_indices==ind]    # get the locations of indices from 0...n
            else:
                sec_2D_points = np.append(sec_2D_points,all_2d_points[point_indices==ind],axis=0)
                sec_3D_points = np.append(sec_3D_points,all_3d_points[point_indices==ind],axis=0)
                BA_indices = np.append(BA_indices,multi_val[point_indices==ind])     # get the locations of indices from 0...n

        return sec_2D_points, sec_3D_points, BA_indices


    def get_BA_data(self,all_3d_points,all_2d_points,point_indices,inc,thresh):
        gv = np.arange(inc - thresh,inc)
        sec_3D_points = np.empty(None)
        sec_2D_points = np.empty(None)
        BA_indices = np.empty(None)          # indices for BA...this is separate from the global indices
               
        for i,ind in enumerate(gv):
            # since you want to get a local set of indices for BA
            multi_val = np.full(all_2d_points.shape[0],i)   #create arrays from 0,1,2,3,...n

            #np.append cant put together multi-dimensional arrays at once,..so I do this
            if i== 0:
                sec_2D_points = all_2d_points[point_indices==ind]
                sec_3D_points = all_3d_points[point_indices==ind]
                BA_indices = multi_val[point_indices==ind]    # get the locations of indices from 0...n
            else:
                sec_2D_points = np.append(sec_2D_points,all_2d_points[point_indices==ind],axis=0)
                sec_3D_points = np.append(sec_3D_points,all_3d_points[point_indices==ind],axis=0)
                BA_indices = np.append(BA_indices,multi_val[point_indices==ind])     # get the locations of indices from 0...n

        return sec_2D_points, sec_3D_points, BA_indices
        

    def get_indices(self,p2d_points,frame_count):   #frame count show the index of the points
    #get indices of 2d points 
        indices = []
        if p2d_points is not None:
            pts_num = p2d_points.shape[0]
            for i in range(pts_num):
                indices.append(frame_count)
            return np.array(indices).ravel()

        # START BUNDLE ADJUSTMENT.
    def BundleAdjustment(self, cam_RT, cam_point_indices, cam_3d_points, cam_2d_points, inc, K,bundle_threshold):
       
        BA_cam_RT = np.array(cam_RT[inc - bundle_threshold : inc]) #get only the latest camera parameters specified by the bundle threshold
        p_indices = np.array(cam_point_indices).ravel()  #convert list to numpy and ravel This contains indices for all the data
        
        BA_cam_2d, BA_cam_3d, BA_indices = self.get_BA_data(cam_3d_points,cam_2d_points,p_indices,inc,bundle_threshold)
        
        n_cameras = bundle_threshold                 #total number of frames used for every bundle adjustment
        n_points = BA_cam_3d.shape[0]      #total number of points obtained

        x0 = np.hstack((BA_cam_RT.ravel(),BA_cam_3d.ravel()))  #DATA input for least square

        #get sparsity matrix for nonlinear least square.
        min_error= self.min_reprojection(x0,n_cameras,n_points,K,BA_indices,BA_cam_2d)
        #create sparsity matrix 
        BA_sparsity = self.create_sparsity_matrix(bundle_threshold,BA_cam_3d.shape[0],BA_indices) 
        #compute non linear least square
        BA_solution = least_squares(self.min_reprojection, x0, jac_sparsity=BA_sparsity, verbose=0, x_scale='jac', ftol=1e-4, method='trf',
        args=(n_cameras, n_points,K, BA_indices, BA_cam_2d))

        result = BA_solution.x    #result of non linear least squares
        
        new_BA_RT = result[:n_cameras * 12].reshape((n_cameras,3, 4))  #12 due to 12 values in R/T --convert back to 3x4 matrix
        new_3d_points = result[n_cameras * 12:].reshape((n_points, 3))

        return new_BA_RT,new_3d_points


    def get_world_points(self,img1,img2,K,RT_p):  
        #pass the two images n and n+1
        a = img1
        b = img2
        Ransac_thresh = 0.775
        match_thresh = 0.7   

        orb = cv2.ORB_create()        # Initiate ORB detector
        kp1, des1 = orb.detectAndCompute(a,None)
        kp2, des2 = orb.detectAndCompute(b,None)

        # create BFMatcher object
        bf = cv2.BFMatcher(cv2.NORM_HAMMING, crossCheck=False)   #pay attention to crossCheck --its false when using bfmatch
        matches = bf.knnMatch(des1,des2,k=2)
        # store all the good matches as per Lowe's ratio test.
    
        good = []
        for m,n in matches:
            if m.distance < match_thresh*n.distance:
                good.append(m)
        # print(len(good),self.frame_inc)
        fimg = cv2.drawMatches(a,kp1,b,kp2,good,None)   #print all features and matches

        #Here to deal with errors
        RT_no_Rot_trans = self.no_RT  #for cases where there is no inlier passed hence no rotation 

        MIN_MATCH_COUNT = 10
        if len(good)>MIN_MATCH_COUNT:
            src_pts = np.float32([ kp1[m.queryIdx].pt for m in good ]).reshape(-1,1,2)
            dst_pts = np.float32([ kp2[m.trainIdx].pt for m in good ]).reshape(-1,1,2)
            
            E, mask = cv2.findEssentialMat(src_pts, dst_pts, K, cv2.RANSAC, Ransac_thresh, 1, None)  #apply ransac and get E matrix
            _, R, T, mask = cv2.recoverPose(E, src_pts, dst_pts, cameraMatrix=K, mask=mask) #get rotation and translation
            matchesMask = mask.ravel().tolist()
            RT = np.column_stack((R,T))   # combine new R and T 

            inlier_src , inlier_dst = [],[]
            for i, m in enumerate(mask):
                if m[0] ==1:
                    inlier_src.append(src_pts[i])  #create keypoints
                    inlier_dst.append(dst_pts[i])
            draw_params = dict(matchColor = (0,255,0), # draw matches in green color
                    singlePointColor = None,
                    matchesMask = matchesMask, # draw only inliers
                    flags = 2)

            fimg = cv2.drawMatches(a,kp1,b,kp2,good,None,**draw_params)   #print just matches

            #both are K because the same intrinsics for the same camera. get the projective matrix
            KR_1 = np.dot(K,RT_p) #this is the initial or previous R and T
            KR_2 = np.dot(K,RT)      #this is the new R and T

            if len(inlier_dst) > 0:
                #This reshape is needed as an input for cv2.triangulate points
                inlier_src = np.array(inlier_src).T.reshape(2,-1)
                inlier_dst = np.array(inlier_dst).T.reshape(2,-1)
                
                world_points = cv2.triangulatePoints(KR_1, KR_2, inlier_src, inlier_dst)
                world_points /= world_points[3]      #divide by 3rd column
            
                wd = world_points.T[:,:3]
                p2d_points = inlier_src.reshape(-1,2)
                return wd, RT, p2d_points,fimg
            else:
                return None,RT_no_Rot_trans,None,fimg   #retrun identity and zero translation if no points detected
    
        else:
            print ("Not enough matches are found - %d/%d" % (len(good),MIN_MATCH_COUNT))
            return None,RT_no_Rot_trans,None,fimg
            



         #    ------------------- CAMERA DATA HANDLING ----------------------------

        #FOR TOP LEFT IMAGE       ----CAMERA 1----
    def camera1_handling(self,K,bundle_threshold,do_bundle=False,verbose=0):
        img1 = self.cam1_frames[self.frame_inc]   #get frames n
        img2 = self.cam1_frames[self.frame_inc+1] #get frames n+1

        try:  #Handling for when there is not enough inlier points
            #get world points, Rotation and Translations and point correspondences
            cam_new_3d, cam_Rot_Trans , cam_p2d_corr_1,cam1_final_img = self.get_world_points(img1,img2,K,self.RT_p1)
            self.RT_p1 = cam_Rot_Trans       #update initial rotation and translation

            if not self.cam1_first_data_done and cam_new_3d is not None:        #assign these variables first before anything else
                self.cam1_3d_points = cam_new_3d
                self.cam1_2d_points = cam_p2d_corr_1
                cam_ind = self.get_indices(cam_p2d_corr_1,self.cam1_inc) #get point indices /index of frame to track points to frame
                self.cam1_point_indices  = cam_ind
                self.cam1_first_data_done = True
                self.cam1_inc = 0            # reset increment to zero  *sometimes not zero at start
    

            if self.cam1_3d_points is not None and cam_new_3d is not None: 
                self.cam1_3d_points= np.append(self.cam1_3d_points,cam_new_3d,axis=0)
                self.cam1_2d_points = np.append(self.cam1_2d_points,cam_p2d_corr_1,axis=0)
                cam_ind = self.get_indices(cam_p2d_corr_1,self.cam1_inc) #get point indices /index of frame to track points to frame
                self.cam1_point_indices = np.append(self.cam1_point_indices,cam_ind)
                
            
            if self.RT_p1.all() != self.no_RT.all():    # only append necessary rotations and translations.
                self.cam1_inc += 1
                self.cam1_RT.append(self.RT_p1)   #add to a list
                if verbose:
                    print("cam1 working....")


            #get only the data needed for the bundle adjustment
            #bundle_threshold - 2 --- to give some space..not for any particular reason
            if (self.cam1_inc > bundle_threshold-2) and (self.cam1_inc % bundle_threshold == 0) and do_bundle:   #for every specified amount of frames do bundle adjustment
                BA_cam_RT,BA_3d =  self.BundleAdjustment(self.cam1_RT, self.cam1_point_indices, self.cam1_3d_points, self.cam1_2d_points, self.cam1_inc, K, bundle_threshold)      
                self.BA_cam_RT.append(BA_cam_RT)
                self.BA_3d.append(BA_3d)
                return BA_3d
            else:
                return cam_new_3d      #else return just the triangulated points
                
        except IndexError as Indx:      # when there are no 3D points to compute pass
            if verbose:
                print("cam1 Error: {0}".format(Indx))
            pass
            #unexplained and unresolved bug. the multiple prints of the index error message is because the code runs
            # the get_world_points function multiple times without updating the image even though every counter for 
            #updating the image is working. This is just an inconvenience and further slows the code down



        #FOR TOP RIGHT IMAGE      ----CAMERA 2----
    def camera2_handling(self,K,bundle_threshold,do_bundle=False,verbose=0):
        img1 = self.cam2_frames[self.frame_inc]   #get frames n
        img2 = self.cam2_frames[self.frame_inc+1] #get frames n+1

        try:  #Handling for when there is not enough inlier points
            #get world points, Rotation and Translations and point correspondences
            cam_new_3d, cam_Rot_Trans , cam_p2d_corr_1,cam2_final_img = self.get_world_points(img1,img2,K,self.RT_p2)
            self.RT_p2 = cam_Rot_Trans       #update initial rotation and translation
            
            if not self.cam2_first_data_done and cam_new_3d is not None:        #assign these variables first before anything else
                self.cam2_3d_points = cam_new_3d
                self.cam2_2d_points = cam_p2d_corr_1
                cam_ind = self.get_indices(cam_p2d_corr_1,self.cam2_inc) #get point indices /index of frame to track points to frame
                self.cam2_point_indices  = cam_ind
                self.cam2_first_data_done = True
                self.cam2_inc = 0            # reset increment to zero  *sometimes not zero at start
    

            if self.cam2_3d_points is not None and cam_new_3d is not None: 
                self.cam2_3d_points= np.append(self.cam2_3d_points,cam_new_3d,axis=0)
                self.cam2_2d_points = np.append(self.cam2_2d_points,cam_p2d_corr_1,axis=0)
                cam_ind = self.get_indices(cam_p2d_corr_1,self.cam2_inc) #get point indices /index of frame to track points to frame
                self.cam2_point_indices = np.append(self.cam2_point_indices,cam_ind)
                
            if self.RT_p2.all() != self.no_RT.all():    # only append necessary rotations and translations.
                self.cam2_inc += 1
                self.cam2_RT.append(self.RT_p2)   #add to a list
                if verbose:
                    print("cam2 working....")

            #get only the data needed for the bundle adjustment
            #bundle_threshold - 2 --- to give some space..not for any particular reason
            if (self.cam2_inc > bundle_threshold-2) and (self.cam2_inc % bundle_threshold == 0) and do_bundle:   #for every specified amount of frames do bundle adjustment
                BA_cam_RT,BA_3d =  self.BundleAdjustment(self.cam2_RT, self.cam2_point_indices, self.cam2_3d_points, self.cam2_2d_points, self.cam2_inc, K, bundle_threshold)      
                self.BA_cam_RT.append(BA_cam_RT)
                self.BA_3d.append(BA_3d)
                return BA_3d
            else:
                return cam_new_3d      #else return just the triangulated points
                
        except IndexError as Indx:      # when there are no 3D points to compute pass
            if verbose:
                print("cam2 Error: {0}".format(Indx))
            pass




        #FOR RIGHT IMAGE             ----CAMERA 3----
    def camera3_handling(self,K,bundle_threshold,do_bundle=False,verbose=0):
        img1 = self.cam3_frames[self.frame_inc]   #get frames n
        img2 = self.cam3_frames[self.frame_inc+1] #get frames n+1

        try:  #Handling for when there is not enough inlier points
            #get world points, Rotation and Translations and point correspondences
            cam_new_3d, cam_Rot_Trans , cam_p2d_corr_1,cam3_final_img = self.get_world_points(img1,img2,K,self.RT_p3)
            self.RT_p3 = cam_Rot_Trans       #update initial rotation and translation
            

            if not self.cam3_first_data_done and cam_new_3d is not None:        #assign these variables first before anything else
                self.cam3_3d_points = cam_new_3d
                self.cam3_2d_points = cam_p2d_corr_1
                cam_ind = self.get_indices(cam_p2d_corr_1,self.cam3_inc) #get point indices /index of frame to track points to frame
                self.cam3_point_indices  = cam_ind
                self.cam3_first_data_done = True
                self.cam3_inc = 0            # reset increment to zero  *sometimes not zero at start
    

            if self.cam3_3d_points is not None and cam_new_3d is not None: 
                self.cam3_3d_points= np.append(self.cam3_3d_points,cam_new_3d,axis=0)
                self.cam3_2d_points = np.append(self.cam3_2d_points,cam_p2d_corr_1,axis=0)
                cam_ind = self.get_indices(cam_p2d_corr_1,self.cam3_inc) #get point indices /index of frame to track points to frame
                self.cam3_point_indices = np.append(self.cam3_point_indices,cam_ind)
                
            
            if self.RT_p3.all() != self.no_RT.all():    # only append necessary rotations and translations.
                self.cam3_inc += 1
                self.cam3_RT.append(self.RT_p3)   #add to a list
                if verbose:
                    print("cam3 working....")


            #get only the data needed for the bundle adjustment
            #bundle_threshold - 2 --- to give some space..not for any particular reason
            if (self.cam3_inc > bundle_threshold-2) and (self.cam3_inc % bundle_threshold == 0) and do_bundle:   #for every specified amount of frames do bundle adjustment
                BA_cam_RT,BA_3d =  self.BundleAdjustment(self.cam3_RT, self.cam3_point_indices, self.cam3_3d_points, self.cam3_2d_points, self.cam3_inc, K, bundle_threshold)      
                self.BA_cam_RT.append(BA_cam_RT)
                self.BA_3d.append(BA_3d)
                return BA_3d
            else:
                return cam_new_3d     #else return just the triangulated points

        except IndexError as Indx:      # when there are no 3D points to compute pass
            if verbose:
                print("cam3 Error: {0}".format(Indx))
            pass


        #FOR BACK RIGHT IMAGE         ----CAMERA 4----
    def camera4_handling(self,K,bundle_threshold,do_bundle =False,verbose=0):
        img1 = self.cam4_frames[self.frame_inc]   #get frames n
        img2 = self.cam4_frames[self.frame_inc+1] #get frames n+1

        try:  #Handling for when there is not enough inlier points
            #get world points, Rotation and Translations and point correspondences
            cam_new_3d, cam_Rot_Trans , cam_p2d_corr_1,cam4_final_img = self.get_world_points(img1,img2,K,self.RT_p4)
            self.RT_p4 = cam_Rot_Trans       #update initial rotation and translation
            
            # final_img = cam4_final_img       #final image to be shown
            # final_img = cv2.resize(final_img,(1080,640))
            # try:
            #     self.pub.publish(self.br.cv2_to_imgmsg(final_img))
            # except CvBridgeError, e:
            #     rospy.logerr("CvBridge Error: {0}".format(e))

            # cv2.imshow("camera_feed",final_img)
            # cv2.waitKey(1)
            # # if cv2.waitKey(1) & 0XFF ==ord("q"):
            # #     break


            if not self.cam4_first_data_done and cam_new_3d is not None:        #assign these variables first before anything else
                self.cam4_3d_points = cam_new_3d
                self.cam4_2d_points = cam_p2d_corr_1
                cam_ind = self.get_indices(cam_p2d_corr_1,self.cam4_inc) #get point indices /index of frame to track points to frame
                self.cam4_point_indices  = cam_ind
                self.cam4_first_data_done = True
                self.cam4_inc = 0            # reset increment to zero  *sometimes not zero at start
    

            if self.cam4_3d_points is not None and cam_new_3d is not None: 
                self.cam4_3d_points= np.append(self.cam4_3d_points,cam_new_3d,axis=0)
                self.cam4_2d_points = np.append(self.cam4_2d_points,cam_p2d_corr_1,axis=0)
                cam_ind = self.get_indices(cam_p2d_corr_1,self.cam4_inc) #get point indices /index of frame to track points to frame
                self.cam4_point_indices = np.append(self.cam4_point_indices,cam_ind)
                
            
            if self.RT_p4.all() != self.no_RT.all():    # only append necessary rotations and translations.
                self.cam4_inc += 1
                self.cam4_RT.append(self.RT_p4)   #add to a list
                if verbose:
                    print("cam4 working....")


            #get only the data needed for the bundle adjustment
            #bundle_threshold - 2 --- to give some space..not for any particular reason
            if (self.cam4_inc > bundle_threshold-2) and (self.cam4_inc % bundle_threshold == 0) and do_bundle:   #for every specified amount of frames do bundle adjustment
                BA_cam_RT,BA_3d =  self.BundleAdjustment(self.cam4_RT, self.cam4_point_indices, self.cam4_3d_points, self.cam4_2d_points, self.cam4_inc, K, bundle_threshold)      
                self.BA_cam_RT.append(BA_cam_RT)
                self.BA_3d.append(BA_3d)
                return BA_3d
            else:
                return cam_new_3d      #else return just the triangulated points
 
        except IndexError as Indx:      # when there are no 3D points to compute pass
            if verbose:
                print("cam4 Error: {0}".format(Indx))
            pass



        #FOR BACK LEFT IMAGE                 ----CAMERA 5------
    def camera5_handling(self,K,bundle_threshold,do_bundle=False,verbose=0):
        img1 = self.cam5_frames[self.frame_inc]   #get frames n
        img2 = self.cam5_frames[self.frame_inc+1] #get frames n+1

        try:  #Handling for when there is not enough inlier points
            #get world points, Rotation and Translations and point correspondences
            cam_new_3d, cam_Rot_Trans , cam_p2d_corr_1,cam5_final_img = self.get_world_points(img1,img2,K,self.RT_p5)
            self.RT_p5 = cam_Rot_Trans       #update initial rotation and translation
            
        
            if not self.cam5_first_data_done and cam_new_3d is not None:        #assign these variables first before anything else
                self.cam5_3d_points = cam_new_3d
                self.cam5_2d_points = cam_p2d_corr_1
                cam_ind = self.get_indices(cam_p2d_corr_1,self.cam5_inc) #get point indices /index of frame to track points to frame
                self.cam5_point_indices  = cam_ind
                self.cam5_first_data_done = True
                self.cam5_inc = 0            # reset increment to zero  *sometimes not zero at start
    

            if self.cam5_3d_points is not None and cam_new_3d is not None: 
                self.cam5_3d_points= np.append(self.cam5_3d_points,cam_new_3d,axis=0)
                self.cam5_2d_points = np.append(self.cam5_2d_points,cam_p2d_corr_1,axis=0)
                cam_ind = self.get_indices(cam_p2d_corr_1,self.cam5_inc) #get point indices /index of frame to track points to frame
                self.cam5_point_indices = np.append(self.cam5_point_indices,cam_ind)
                
            
            if self.RT_p5.all() != self.no_RT.all():    # only append necessary rotations and translations.
                self.cam5_inc += 1
                self.cam5_RT.append(self.RT_p5)   #add to a list
                if verbose:
                    print("cam5 working....")


            #get only the data needed for the bundle adjustment
            #bundle_threshold - 2 --- to give some space..not for any particular reason
            if (self.cam5_inc > bundle_threshold-2) and (self.cam5_inc % bundle_threshold == 0) and do_bundle:   #for every specified amount of frames do bundle adjustment
                BA_cam_RT,BA_3d =  self.BundleAdjustment(self.cam5_RT, self.cam5_point_indices, self.cam5_3d_points, self.cam5_2d_points, self.cam5_inc, K, bundle_threshold)      
                self.BA_cam_RT.append(BA_cam_RT)
                self.BA_3d.append(BA_3d)
                return BA_3d
            else:
                return cam_new_3d      #else return just the triangulated points

        except IndexError as Indx:      # when there are no 3D points to compute pass
            if verbose:
                print("cam5 Error: {0}".format(Indx))
            pass
        




        #FOR LEFT IMAGE             ----CAMERA 6------
    def camera6_handling(self,K,bundle_threshold,do_bundle=False,verbose=0):
        img1 = self.cam6_frames[self.frame_inc]   #get frames n
        img2 = self.cam6_frames[self.frame_inc+1] #get frames n+1

        try:  #Handling for when there is not enough inlier points
            #get world points, Rotation and Translations and point correspondences
            cam_new_3d, cam_Rot_Trans , cam_p2d_corr_1,cam6_final_img = self.get_world_points(img1,img2,K,self.RT_p6)
            self.RT_p6 = cam_Rot_Trans       #update initial rotation and translation
            
        
            if not self.cam6_first_data_done and cam_new_3d is not None:        #assign these variables first before anything else
                self.cam6_3d_points = cam_new_3d
                self.cam6_2d_points = cam_p2d_corr_1
                cam_ind = self.get_indices(cam_p2d_corr_1,self.cam6_inc) #get point indices /index of frame to track points to frame
                self.cam6_point_indices  = cam_ind
                self.cam6_first_data_done = True
                self.cam6_inc = 0            # reset increment to zero  *sometimes not zero at start
    

            if self.cam6_3d_points is not None and cam_new_3d is not None: 
                self.cam6_3d_points= np.append(self.cam6_3d_points,cam_new_3d,axis=0)
                self.cam6_2d_points = np.append(self.cam6_2d_points,cam_p2d_corr_1,axis=0)
                cam_ind = self.get_indices(cam_p2d_corr_1,self.cam6_inc) #get point indices /index of frame to track points to frame
                self.cam6_point_indices = np.append(self.cam6_point_indices,cam_ind)
                
            
            if self.RT_p6.all() != self.no_RT.all():    # only append necessary rotations and translations.
                self.cam6_inc += 1
                self.cam6_RT.append(self.RT_p6)   #add to a list
                if verbose:
                    print("Cam6 working....")


            #get only the data needed for the bundle adjustment
            #bundle_threshold - 2 --- to give some space..not for any particular reason
            if (self.cam6_inc > bundle_threshold-2) and (self.cam6_inc % bundle_threshold == 0) and do_bundle:   #for every specified amount of frames do bundle adjustment
                BA_cam_RT,BA_3d =  self.BundleAdjustment(self.cam6_RT, self.cam6_point_indices, self.cam6_3d_points, self.cam6_2d_points, self.cam6_inc, K, bundle_threshold)      
                self.BA_cam_RT.append(BA_cam_RT)
                self.BA_3d.append(BA_3d)
                return BA_3d
            else:
                return cam_new_3d      #else return just the triangulated points
                
            
        except IndexError as Indx:      # when there are no 3D points to compute pass
            if verbose:
                print("Cam6 Error: {0}".format(Indx))
            pass

       


       


         

#/------------------------------------------------------------------------------------------------------------------/
                                            # MAIN LOOP
    def image_analyis(self):
        print("Starting Structure From Motion Code.........")
        print("Loading and undistorting images from the cameras......")
        
        while not rospy.is_shutdown():
            #put all the images in an array
            active = np.array([(self.top_left_image > 0),(self.top_right_image > 0),
            (self.left_image > 0), (self.right_image > 0), (self.back_left_image > 0), (self.back_right_image> 0)])

            cond = active.shape    # get the shape... if an image isn't present len(cond) = 1
            init_frame_threshold = 10       #number of frames to have before starting any analysis
            bundle_threshold = 15           #Do a bundle adjustment every 50 frames

            if len(cond) == 4:   #if all images are active 
                #assign images from self object
                top_left_img = self.top_left_image
                top_right_img = self.top_right_image
                right_img = self.right_image
                left_img = self.left_image
                back_right_img = self.back_right_image
                back_left_img = self.back_left_image
                
                #undistort images
                undist_top_left_img = self.undistort(top_left_img,K1,D1)
                undist_top_right_img = self.undistort(top_right_img,K2,D2)
                undist_right_img = self.undistort(right_img,K3,D3)
                undist_left_img = self.undistort(left_img,K4,D4)
                undist_back_left_img = self.undistort(back_left_img,K5,D5)
                undist_back_right_img = self.undistort(back_right_img,K6,D6)

                # keep image in an image array to run in background
                self.cam1_frames.append(undist_top_left_img)
                self.cam2_frames.append(undist_top_right_img)
                self.cam3_frames.append(undist_right_img)
                self.cam4_frames.append(undist_back_right_img)
                self.cam5_frames.append(undist_back_left_img)
                self.cam6_frames.append(undist_left_img)
                
                thresh_condition = (len(self.cam1_frames) > init_frame_threshold) and (len(self.cam2_frames) > init_frame_threshold)\
                and (len(self.cam3_frames) > init_frame_threshold) and (len(self.cam4_frames) > init_frame_threshold) \
                and (len(self.cam5_frames) > init_frame_threshold) and (len(self.cam6_frames) > init_frame_threshold)

                # #start run when all frames are above threshold
                if thresh_condition:
                    # RUN CALCULATIONS AND PLOTS
                    do_bundle = True    #perform bundle adjustment -- boolean

                    new1_3d = self.camera1_handling(K1,bundle_threshold,do_bundle=do_bundle)
                    new2_3d = self.camera2_handling(K2,bundle_threshold,do_bundle=do_bundle)
                    new3_3d = self.camera3_handling(K3,bundle_threshold,do_bundle=do_bundle)
                    new4_3d = self.camera4_handling(K4,bundle_threshold,do_bundle=do_bundle)
                    new5_3d = self.camera5_handling(K5,bundle_threshold,do_bundle=do_bundle)
                    new6_3d = self.camera6_handling(K6,bundle_threshold,do_bundle=do_bundle)

                    print("Publishing....")
                    #print the different image increments of the different cameras.
                    print("cam1:",self.cam1_inc,"cam2:",self.cam2_inc,"cam3:",self.cam3_inc,"cam4:",self.cam4_inc,"cam5:",self.cam5_inc,"cam6:",self.cam6_inc)

                    # if there are 3d points available
                    try:    #Generate and publish point clouds...point clouds are published in the camera frame and assembled with assemble scanner
                        cam1_scan = self.generate_point_cloud(new1_3d, new1_3d.shape[0], rospy.Time.now(),"cam_top_left","odom")
                        self.cam1_pointcloud.publish(cam1_scan)
                        self.display_pointcloud()

                    except ValueError as val:        #minor error caused by no matches detected, no important effect.
                        print("value error cam 1: {0}".format(val))
                        pass
                    except AttributeError as att:        #minor error caused by no matches detected, no important effect.
                        # print("attribute error cam 5: {0}".format(att))
                        pass



                    #//////////////////////////////////////////////////////////////////////////////////////////////////////////
                    try:      #Generate and publish point clouds...point clouds are published in the camera frame and assembled with assemble scanner
                        cam2_scan = self.generate_point_cloud(new2_3d, new2_3d.shape[0], rospy.Time.now(),"cam_top_right","odom")
                        self.cam2_pointcloud.publish(cam2_scan)
                        self.display_pointcloud()
                        
                    except ValueError as val:        #minor error caused by no matches detected, no important effect.
                        print("value error cam 2: {0}".format(val))
                        pass
                    except AttributeError as att:        #minor error caused by no matches detected, no important effect.
                        # print("attribute error cam 2: {0}".format(att))
                        pass


                    #//////////////////////////////////////////////////////////////////////////////////////////////////////////
                    try:  #Generate and publish point clouds...point clouds are published in the camera frame and assembled with assemble scanner
                        cam3_scan = self.generate_point_cloud(new3_3d, new3_3d.shape[0], rospy.Time.now(),"cam_right","odom")
                        self.cam3_pointcloud.publish(cam3_scan)
                        self.display_pointcloud()    
                 
                    except ValueError as val:        #minor error caused by no matches detected, no important effect.
                        print("value error cam 3: {0}".format(val))
                        pass
                    except AttributeError as att:        #minor error caused by no matches detected, no important effect.
                        # print("attribute error cam 3: {0}".format(att))
                        pass

                    #//////////////////////////////////////////////////////////////////////////////////////////////////////////
                    try:  #Generate and publish point clouds...point clouds are published in the camera frame and assembled with assemble scanner
                        cam4_scan = self.generate_point_cloud(new4_3d, new4_3d.shape[0], rospy.Time.now(),"cam_back_right","odom")
                        self.cam4_pointcloud.publish(cam4_scan)
                        self.display_pointcloud()
                         
                    except ValueError as val:        #minor error caused by no matches detected, no important effect.
                        print("value error cam 4: {0}".format(val))
                        pass
                    except AttributeError as att:        #minor error caused by no matches detected, no important effect.
                        # print("attribute error cam 4: {0}".format(att))
                        pass


                    #//////////////////////////////////////////////////////////////////////////////////////////////////////////
                    try:  #Generate and publish point clouds...point clouds are published in the camera frame and assembled with assemble scanner
                        cam5_scan = self.generate_point_cloud(new5_3d, new5_3d.shape[0], rospy.Time.now(),"cam_back_left","odom")
                        self.cam5_pointcloud.publish(cam5_scan)
                        self.display_pointcloud()

                    except ValueError as val:        #minor error caused by no matches detected, no important effect.
                        print("value error cam 5: {0}".format(val))
                        pass
                    except AttributeError as att:        #minor error caused by no matches detected, no important effect.
                        # print("attribute error cam 5: {0}".format(att))
                        pass


                    #//////////////////////////////////////////////////////////////////////////////////////////////////////////
                    try:  #Generate and publish point clouds...point clouds are published in the camera frame and assembled with assemble scanner
                        cam6_scan = self.generate_point_cloud(new6_3d, new6_3d.shape[0], rospy.Time.now(),"cam_left","odom")
                        self.cam6_pointcloud.publish(cam6_scan)
                        self.display_pointcloud()
                         
                    except ValueError as val:        #minor error caused by no matches detected, no important effect.
                        print("value error cam 6: {0}".format(val))
                        pass
                    except AttributeError as att:        #minor error caused by no matches detected, no important effect.
                        # print("attribute error cam 6: {0}".format(att))
                        pass

                    #once everything is done increase frame count
                    self.frame_inc += 1            #increment frame counter to move to next images
                
                    
                    

def main(args):
    rospy.init_node('cam_node', anonymous=True)
    work_env_cam = sfm()
    work_env_cam.image_analyis()
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")
    cv2.destroyAllWindows()


if __name__ == '__main__':
    main(sys.argv)