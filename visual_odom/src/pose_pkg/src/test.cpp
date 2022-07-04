#include<stdio.h>
#include <iostream>
#include <math.h>
#include<nav_msgs/Odometry.h>
#include<vector>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>

#include<geometry_msgs/PoseStamped.h>
#include<geometry_msgs/PoseWithCovarianceStamped.h>

#include "tf/tf.h"

#include <opencv2/opencv.hpp>
#include <opencv2/highgui.hpp>

#include<opencv2/aruco.hpp>
#include<ros/ros.h>

#include<eigen3/Eigen/Dense>
#include <pcl/common/transforms.h>

#include<time.h>
#include <typeinfo>

#include<tf/LinearMath/Quaternion.h>



//float arucos_t_mat[] = {
//    0,0.337,0.21,
//    0.215,0.655,0.145
//};

//float arucos_rot_mat[]={
//    0,0,1, 1,0,0, 0,1,0,
//    0,0,1, 1,0,0, 0,1,0,

//};

float arucos_rot_mat[180] = {
    // a
    -1,0,0, 0,0,1, 0,1,0,//o1
    // b
    0.707,0,-0.707, -0.707,0,-0.707, 0,1,0,
    // c
    -1,0,0, 0,0,1, 0,1,0,
    // d
    -0.707,0,0.707, 0.707,0,0.707, 0,1,0,
    // e
    1,0,0, 0,0,-1, 0,1,0,//o2
    // f
    0,0,1, 1,0,0, 0,1,0,//o3
    // g
    0,0,-1, -1,0,0, 0,1,0,//o4
    // h
    -1,0,0, 0,0,1, 0,1,0,
    // i
    -1,0,0, 0,0,1, 0,1,0,
    // j //not in our dictionary
    // k
    0.707,0,0.707, 0.707,0,-0.707, 0,1,0,
    // l
    0,0,1, 1,0,0, 0,1,0,
    // m //not in our dictionary
    // n
    -1,0,0, 0,0,1, 0,1,0,
    // o
    0,0,-1, -1,0,0, 0,1,0,
    // p //not in our dictionary
    // q
    0,0,1, 1,0,0, 0,1,0,
    // r
    0,0,-1, -1,0,0, 0,1,0,
    // s //not in our dictionary
    // t
    -0.707,0,-0.707, -0.707,0,0.707, 0,1,0,
    // u
    0,0,1, 1,0,0, 0,1,0,
    // v
    0,0,-1, -1,0,0, 0,1,0,
    // w
    0,0,1, 1,0,0, 0,1,0,
    // x

};

float arucos_t_mat[60] = {

    // a
    0,0,0,
    // b
    1.319,2.486,0.14,
    // c
    0,0,0,
    // d
    1.444,2.652,0.14,
    // e
    -0.01,5.88,0.102,
    // f
    0,0.225,0.11,
    // g
    2.645,2.51,0.104,
    // h
    0,0,0,
    // i
    0,0,0,
    // j //not in our dictionary
    // k
    1.471,2.509,0.151,
    // l
    0.085,2.472,0.105,
    // m //not in our dictionary
    // n
    0,0,0,
    // o
    1.325,0.808,0.163,
    // p //not in our dictionary
    // q
    2.68,5.661,0.113,
    // r
    2.52,5.655,0.109,
    // s //not in our dictionary
    // t
    1.295,2.636,0.138,
    // u
    1.488,0.806,0.105,
    // v
    1.32,4.914,0.108,
    // w
    1.48,4.923,0.112,
    // x
};

int marker_bits[]=
{
    // a
    0,0,1,0,0, 0,1,0,1,0, 1,1,1,1,1, 1,0,0,0,1, 1,0,0,0,1,
    //#b
    1,1,1,1,0, 1,0,0,0,1, 1,1,1,1,0, 1,0,0,0,1, 1,1,1,1,0,
    //#c
    0,1,1,1,0, 1,0,0,0,1, 1,0,0,0,0, 1,0,0,0,1, 0,1,1,1,0,
    //#d
    1,1,1,1,0, 1,0,0,0,1, 1,0,0,0,1, 1,0,0,0,1, 1,1,1,1,0,
    //#e
    1,1,1,1,1, 1,0,0,0,0, 1,1,1,1,0, 1,0,0,0,0, 1,1,1,1,1,
    //#f
    1,1,1,1,1, 1,0,0,0,0, 1,1,1,1,0, 1,0,0,0,0, 1,0,0,0,0,
    //#g
    0,1,1,1,0, 1,0,0,0,0, 1,0,1,1,1, 1,0,0,0,1, 0,1,1,1,0,
    //#h
    1,0,0,0,1, 1,0,0,0,1, 1,1,1,1,1, 1,0,0,0,1, 1,0,0,0,1,
    //#i
    1,1,1,1,1, 0,0,1,0,0, 0,0,1,0,0, 0,0,1,0,0, 1,1,1,1,1,
    // #k
    1,0,0,0,1, 1,0,0,1,0, 1,1,1,0,0, 1,0,0,1,0, 1,0,0,0,1,
    // #l
    1,0,0,0,0, 1,0,0,0,0, 1,0,0,0,0, 1,0,0,0,0, 1,1,1,1,1,
    // #n
    1,0,0,0,1, 1,1,0,0,1, 1,0,1,0,1, 1,0,0,1,1, 1,0,0,0,1,
    // #o
    0,1,1,1,0, 1,0,0,0,1, 1,0,0,0,1, 1,0,0,0,1, 0,1,1,1,0,
    //#q
    0,1,1,1,0, 1,0,0,0,1, 1,0,0,0,1, 1,0,0,1,0, 0,1,1,0,1,
    //#r
    1,1,1,1,0, 1,0,0,0,1, 1,1,1,1,0, 1,0,0,1,0, 1,0,0,0,1,
    //#t
    1,1,1,1,1, 0,0,1,0,0, 0,0,1,0,0, 0,0,1,0,0, 0,0,1,0,0,
    //#u
    1,0,0,0,1, 1,0,0,0,1, 1,0,0,0,1, 1,0,0,0,1, 0,1,1,1,0,
    //#v
    1,0,0,0,1, 1,0,0,0,1, 1,0,0,0,1, 0,1,0,1,0, 0,0,1,0,0,
    //#w
    1,0,0,0,1, 1,0,0,0,1, 1,0,1,0,1, 1,0,1,0,1, 0,1,0,1,0,
    //#x
    1,0,0,0,1, 0,1,0,1,0, 0,0,1,0,0, 0,1,0,1,0, 1,0,0,0,1,
};
double camera_matrix[]={
    1074.03,0.0,640.0,0.0,1074.080,360.0,0.0,0.0,1.0
};
//double camera_matrix2[]={
//    1112.03, 0.0, 640.0, 0.0, 1124.28, 360.0, 0.0, 0.0, 1.0
//};
double distortion_matrix[]{
    0.0777, -0.1403,
    0.0021, -0.0154,
    0.05011
};
//double distortion_matrix2[]{
//    0.0777, -0.1403,
//    0.0021, -0.0154,
//    0.05011
//};

    geometry_msgs::PoseStamped msg_required;

  void ekf_pose_callback(geometry_msgs::PoseWithCovarianceStampedConstPtr msg)
  {
         std::cout<<"hiiiiii ";
        msg_required.header.frame_id="/map";
        msg_required.header.stamp=ros::Time::now();
        msg_required.pose.position.x=msg->pose.pose.position.x;
        msg_required.pose.position.y=msg->pose.pose.position.y;
        msg_required.pose.position.z=msg->pose.pose.position.z;
        msg_required.pose.orientation.x=msg->pose.pose.orientation.x;
        msg_required.pose.orientation.y=msg->pose.pose.orientation.y;
        msg_required.pose.orientation.z=msg->pose.pose.orientation.z;
        msg_required.pose.orientation.w=msg->pose.pose.orientation.w;

    }


void getTransform( Eigen::Affine3f &world_aruco_tf, int id){ // returns tf of aruco marker in world frame

    world_aruco_tf.matrix()<<arucos_rot_mat[9*id+0], arucos_rot_mat[9*id+1], arucos_rot_mat[9*id+2], arucos_t_mat[3*id+0],
                             arucos_rot_mat[9*id+3], arucos_rot_mat[9*id+4], arucos_rot_mat[9*id+5], arucos_t_mat[3*id+1],
                             arucos_rot_mat[9*id+6], arucos_rot_mat[9*id+7], arucos_rot_mat[9*id+8], arucos_t_mat[3*id+2],
                             0, 0, 0, 1;
}

void get_camera_wc( cv::Vec3d& rvec,  cv::Vec3d& tvec, int id, float* op_arr)
{

    Eigen::Affine3f world_aruco_tf = Eigen::Affine3f::Identity(); //tf of detected aruco marker in world frame
    getTransform(world_aruco_tf, id);
//    for (int i = 0; i<4; i++) // debug call
//    {
//        for (int j = 0; j<4; j++)
//            std::cout << world_aruco_tf(i,j) << " ";
//        std:: cout << "\n";
//    }

//-----------camera_pose start----------------------

    cv::Mat r_matrix;
    cv::Rodrigues(rvec, r_matrix);//rvec to rotation matrix //rvec:camera to aruco marker

    //initialising camera_aruco_tf using rotation matrix and x y z of marker in camera frame
    Eigen::Affine3f camera_aruco_tf = Eigen::Affine3f::Identity();
    for(int k=0;k<3;k++)
        for(int j=0;j<3;j++)
            camera_aruco_tf(k,j) = ((double*)r_matrix.data)[k*3+j];
    for(int k=0;k<3;k++)
        camera_aruco_tf(k,3) = tvec[k];

    Eigen::Affine3f aruco_camera_tf = camera_aruco_tf.inverse();

    //tf of camera in detected aruco marker frame

    Eigen::Affine3f world_camera_tf = world_aruco_tf * aruco_camera_tf;//tf(c,w) = tf(am,w)*tf(c,am)

    float roll=0,pitch=0,yaw=0;
    pcl::getEulerAngles(world_camera_tf,roll,pitch,yaw);

    //x y z of camera in world frame
    for( int i=0; i<3; i++){
        op_arr[i] = world_camera_tf(i,3);
    }
    op_arr[3] = roll;
    op_arr[4] = pitch;
    op_arr[5] = yaw;

//    std::cout<<op_arr[0]<<" "<<op_arr[1]<<" "<<op_arr[2]<<" ----   "<<roll*180/3.14<<" "<<pitch*180/3.14<<" "<<yaw*180/3.14<<"\n";
    //rotate world in rpy order wrt to fix coordinate system of world
}

void pose_estimation(cv::Mat& frame, cv::Ptr<cv::aruco::Dictionary>& aruco_dict,
                        cv::Mat& K, cv::Mat& D,
                        std::vector<cv::Vec3d>& rvecs, std::vector<cv::Vec3d>& tvecs,
                        float* final_xyzrpy)
{
    cv::Mat gray;
    cv::cvtColor(frame,gray, cv::COLOR_BGR2GRAY);

//    cv::boxFilter(gray, gray, -1, cv::Size(3,3));
    cv::GaussianBlur(gray, gray, cv::Size(3,3),3,3);
//    cv::boxFilter(gray, gray, -1, cv::Size(3,3));
//    cv::boxFilter(gray, gray, -1, cv::Size(3,3));
    //   cv::bilateralFilter(frame, gray, -1, 3, 3);

    cv::Mat sharpeningKernel(3,3,CV_8SC1);

    sharpeningKernel.data[0] = 0;
    sharpeningKernel.data[1] = -1;
    sharpeningKernel.data[2] = 0;
    sharpeningKernel.data[3] = -1;
    sharpeningKernel.data[4] = 5;
    sharpeningKernel.data[5] = -1;
    sharpeningKernel.data[6] = 0;
    sharpeningKernel.data[7] = -1;
    sharpeningKernel.data[8] = 0;

    cv::filter2D(gray, gray, -1, sharpeningKernel);

    //   sharpeningKernel = np.array(([0, -1, 0],[-1, 5, -1],[0, -1, 0]), dtype="int")
    //   filter2D is used to perform the convolution.
    //   The third parameter (depth) is set to -1 which means the bit-depth of the output image is the
    //   same as the input image. So if the input image is of type CV_8UC3, the output image will also be of the same type

    cv::Ptr<cv::aruco::DetectorParameters> arucoParams = cv::aruco::DetectorParameters::create();
    arucoParams->errorCorrectionRate=0.1;
    arucoParams->doCornerRefinement = true;
    arucoParams->adaptiveThreshWinSizeMin=21;

    std::vector<std::vector<cv::Point2f> > corners;
    cv::Mat marker_ids;
    std::cout<<"here\n";

    cv::aruco::detectMarkers(gray, aruco_dict, corners, marker_ids, arucoParams);
    std::cout<<"here1\n";

    int n_markers = marker_ids.rows * marker_ids.cols;

    float camera_infos_worldFrame[n_markers][6];//stores camera x,y,z,roll,pitch,yaw returned by all visible markers n

    std::vector<int> ids;//ye sahiaa rha hai:)
    for(int i=0;i<n_markers;i++)
        ids.push_back(((int*)marker_ids.data)[i]);

    for(int i=0;i<n_markers;i++)
    {
        cv::Rect rect = cv::boundingRect(corners[i]);
        std::cout<<"here2\n";
        cv::rectangle(frame, rect, cv::Scalar(0,255,0),2);
    }

    if(ids.size()>0)
    {
        float op_arr[6];//return x,y,z,roll,pitch,yaw of camera in world frame (processed by individual markers)
//        cv::aruco::drawDetectedMarkers(frame, corners, marker_ids);
        cv::aruco::estimatePoseSingleMarkers(corners, 0.15, K, D,  rvecs,  tvecs );
        std::cout<<"here3\n";
        for (int i=0;i<ids.size();i++)
        {
            cv::Vec3d rvec=rvecs[i];
            cv::Vec3d tvec=tvecs[i];//ye sahiaa rha hai:)

            get_camera_wc(rvec, tvec, ids[i], op_arr);
            std::cout<<"here4\n";

            camera_infos_worldFrame[i][0] = op_arr[0];
            camera_infos_worldFrame[i][1] = op_arr[1];
            camera_infos_worldFrame[i][2] = op_arr[2];
            camera_infos_worldFrame[i][3] = op_arr[3];
            camera_infos_worldFrame[i][4] = op_arr[4];
            camera_infos_worldFrame[i][5] = op_arr[5];
            cv::aruco::drawAxis(frame,K,D,rvec,tvec,0.05 );
        }
    }

    if(ids.size()>0)
    {
        float sum[6]={0,0,0,0,0,0};

        for (int i=0;i<ids.size();i++)  // give final x,y,z,roll,pitch,yaw of camera in world frame by averaging out all data given by individual markers
        {
            for (int j=0; j<6; j++)
                sum[j] += camera_infos_worldFrame[i][j];
        }
        for (int j=0; j<6; j++)
            final_xyzrpy[j] = sum[j]/ids.size();
        //std::cout<<final_xyzrpy[0]<<" "<<final_xyzrpy[1]<<" "<<final_xyzrpy[2]<<" ----   "<<final_xyzrpy[3]*180/3.14<<" "<<final_xyzrpy[4]*180/3.14<<" "<<final_xyzrpy[5]*180/3.14<<"\n";
    }
}

int main(int argc, char** argv)
{
    ros::init(argc, argv,"aruco_cpp");

    ros::NodeHandle nh;

//    ros::Subscriber sub = nh.subscribe

    ros::AsyncSpinner aspinner(4);

    ros::Publisher ekf_pub=nh.advertise<geometry_msgs::PoseStamped>("ekf_pose", 10);
      ros::Publisher vo_pub=nh.advertise<nav_msgs::Odometry>("vo", 10);
      ros::Subscriber sub = nh.subscribe("/robot_pose_ekf/odom_combined", 10,ekf_pose_callback);
//checking camera index
//    {
//        cv::VideoCapture video(2);

//        while(1)
//        {
//            cv::Mat frame;
//            video >> frame;

//            cv::imshow("temp",frame);
//            cv::waitKey(1);

//        }

//    }
//    return 0;

    int cam1_id = 4;
    int cam2_id = 0;

    int n_markers = 20;
    int marker_size = 5;

    cv::Ptr<cv::aruco::Dictionary> aruco_dict =cv::aruco::generateCustomDictionary(n_markers, marker_size);
    cv::Mat& bytes_list = aruco_dict->bytesList;

    cv::Mat bits = cv::Mat(marker_size, marker_size, CV_8UC1);

    for(int i=0;i<n_markers;i++)
    {
        int marker_size_count = marker_size * marker_size;
        for(int j=0;j<marker_size_count;j++)
            bits.data[j] = marker_bits[i*marker_size_count + j];

        cv::Mat byte_list = cv::aruco::Dictionary::getByteListFromBits(bits);

        for(int j=0;j<4;j++)
            ((int*)bytes_list.data)[i*4+j] = ((int*)byte_list.data)[j];
    }


    cv::VideoCapture video(cam1_id);
//    video.set(cv::CAP_PROP_FRAME_HEIGHT, 480);
//    video.set(cv::CAP_PROP_FRAME_WIDTH, 640);

    video.set(cv::CAP_PROP_FRAME_HEIGHT, 720);
    video.set(cv::CAP_PROP_FRAME_WIDTH, 1280);

    video.set(cv::CAP_PROP_FPS, 120);
    video.set(cv::CAP_PROP_FOURCC, CV_FOURCC('M','J','P','G'));

//    cv::VideoCapture video2(cam2_id);
//    video2.set(cv::CAP_PROP_FRAME_HEIGHT, 480);
//    video2.set(cv::CAP_PROP_FRAME_WIDTH, 640);
//    video2.set(cv::CAP_PROP_FPS, 60);
//    video2.set(cv::CAP_PROP_FOURCC, CV_FOURCC('M','J','P','G'));

    //    cv::Mat K2 = cv::Mat(3,3,CV_64FC1);
    //    for(int j=0;j<9;j++)
    //        ((double*)K2.data)[j] = camera_matrix[j];

    //    cv::Mat D2 = cv::Mat(1,5,CV_64FC1);
    //    for(int j=0;j<5;j++)
    //        ((double*)D.data)[j] = distortion_matrix2[j];

    cv::Mat K;
    cv::Mat D;

//    cv::Mat K2;
//    cv::Mat D2;

    cv::FileStorage fs1("/home/ayush/monocular/cam1/camera_parameters_1280x720.txt",cv::FileStorage::READ);
//    cv::FileStorage fs2("/home/nuc/monocular/cam2/camera_parameters_1280x720.txt",cv::FileStorage::READ);

  //  cv::FileStorage fs1("/home/nuc/monocular/cam1/camera_parameters_640x480.txt",cv::FileStorage::READ);
//    cv::FileStorage fs2("/home/nuc/monocular/cam2/camera_parameters_640x480.txt",cv::FileStorage::READ);

    fs1["K_left"] >> K;
    fs1["D_left"] >> D;

//    fs2["K_left"] >> K2;
//    fs2["D_left"] >> D2;

//    pcl::console::TicToc tt;



    aspinner.start();

    while(ros::ok())
    {
//        tt.tic();

        cv::Mat frame;
        video >> frame;
//        cv::Mat frame2;
//        video2 >> frame2;

        //        frame=cv2.resize(frame,(640,480),interpolation=cv2.INTER_LINEAR)

        std::vector<cv::Vec3d> rvec;
        std::vector<cv::Vec3d> tvec;

//        std::vector<cv::Vec3d> rvec2;
//        std::vector<cv::Vec3d> tvec2;

        float final_xyzrpy[6];
       //float final2_xyzw[4];

        pose_estimation(frame, aruco_dict, K, D, rvec, tvec, final_xyzrpy);

        //pose_estimation(frame2, aruco_dict, K2, D2, rvec2, tvec2, final2_xyzw);
        cv::imshow("Estimated Pose", frame);

//        if (tvec.size())
        //std::cout<<tvec[0]<<"\n";

//        cv::resize(frame,frame, cv::Size(0,0),0.5,0.5);
//        cv::resize(frame2, frame2, cv::Size(0,0),0.5,0.5);

        float roll = final_xyzrpy[3];
        float pitch = final_xyzrpy[4];
        float yaw = final_xyzrpy[5];
        std::cout<<final_xyzrpy[0]<<" "<<final_xyzrpy[1]<<" "<<final_xyzrpy[2]<<" "<<roll*180/3.14<<" "<< pitch*180/3.14<<" "<<yaw*180/3.14<<"\n";

//        tf::Quaternion quat(yaw, pitch,roll);
        Eigen::Affine3f cTw;
        pcl::getTransformation( final_xyzrpy[0], final_xyzrpy[1], final_xyzrpy[2], roll, pitch, yaw, cTw);
        Eigen::Affine3f bTc,bTw;
        bTc.matrix()<<0,-1,0,0,  0,0,-1,0,  1,0,0,0,  0,0,0,1;
        bTw= cTw * bTc;
        Eigen::Quaternionf orientation = Eigen::Quaternionf(bTw.rotation());
        nav_msgs::Odometry odom;
//        geometry_msgs::PoseStamped pose_stamped;
            geometry_msgs::PoseWithCovariance pose_vo;
            geometry_msgs::Pose pose;
        pose.position.x = final_xyzrpy[0];
        pose.position.y = final_xyzrpy[1];
        pose.position.z = final_xyzrpy[2];
        //std::cout<<roll*180/3.14<<" "<<pitch*180/3.14<<" "<<yaw*180/3.14<<"\n";
         pose.orientation.x = orientation.coeffs()(0);
         pose.orientation.y = orientation.coeffs()(1);
         pose.orientation.z = orientation.coeffs()(2);
         pose.orientation.w = orientation.coeffs()(3);
         odom.header.frame_id="/map";
         odom.child_frame_id="/base_link";

         odom.header.stamp = ros::Time::now();
         pose_vo.pose=pose;
         odom.pose=pose_vo;
         odom.pose.covariance={0.1, 0, 0, 0, 0, 0,
                               0, 0.1, 0, 0, 0, 0,
                               0, 0, 0.1, 0, 0, 0,
                               0, 0, 0, 0.1, 0, 0,
                               0, 0, 0, 0, 0.1, 0,
                               0, 0, 0, 0, 0, 0.1};
         vo_pub.publish(odom);
            //msg_required = ros::topic::waitForMessage("/robot_pose_ekf/odom_combined", create_path);
        ekf_pub.publish(msg_required);


//        tf::Transform transform;
//        transform.setOrigin( tf::Vector3(final_xyzrpy[0], final_xyzrpy[1], final_xyzrpy[2]) );
//        tf::Quaternion q;
//        q.setRPY(final_xyzrpy[3],final_xyzrpy[4],final_xyzrpy[5]);
//        transform.setRotation(q);
//        static tf::TransformBroadcaster br;
//        br.sendTransform(tf::StampedTransform(transform.inverse(), ros::Time::now(),"/base_link","/map"));
     //  ros::Duration(5, 0).sleep();
//       tf::TransformListener listener;
//        tf::StampedTransform ocTbl;     //tranform of odom_combined in base link frame
//        std::cout<<"finally here\n";
//        listener.lookupTransform ("/odom_combined","/base_link",ros::Time(0), ocTbl);
      //  std::cout<<"finally here\n";
//        std::cout<<"finally here2\n";
//        tf::StampedTransform blTmap; //tranform of base_link in map frame
//         tf::Quaternion orientation_f;

//         orientation_f.setX(orientation.coeffs()(0));
//         orientation_f.setY(orientation.coeffs()(1));
//         orientation_f.setZ(orientation.coeffs()(2));
//         orientation_f.setW(orientation.coeffs()(3));



//         blTmap.setRotation(orientation_f);
//         tf::Vector3 ftvec= {final_xyzrpy[0], final_xyzrpy[1], final_xyzrpy[2]};
//         blTmap.setOrigin(ftvec);

//         // tf::StampedTransform ocTmap;
//          tf::Transform yeh=ocTbl * blTmap;
//    //     ocTmap.Transform = ocTbl * blTmap;
////          orientation_f=ocTmap.Transform.getRotation();
//          orientation_f=yeh.getRotation();
////          tf::Vector3 tvec_ocTmap=ocTmap.getOrigin();
//          tf::Vector3 tvec_ocTmap=yeh.getOrigin();

//      float ekf_xyzrpy[6];
//          ekf_xyzrpy[0]=tvec_ocTmap[0];
//          ekf_xyzrpy[1]=tvec_ocTmap[1];
//          ekf_xyzrpy[2]=tvec_ocTmap[2];

//         ros::Publisher ekf_pose_pub=nh.advertise<geometry_msgs::PoseStamped>("ekf_pose", 1000);
//          geometry_msgs::PoseStamped ekf_stamped;
//          ekf_stamped.header.stamp = ros::Time().now();
//          ekf_stamped.header.frame_id = "/ekf_pose";
//         geometry_msgs::PoseWithCovariance ekf_vo;
//          pose.position.x = ekf_xyzrpy[0];
//          pose.position.y = ekf_xyzrpy[1];
//          pose.position.z = ekf_xyzrpy[2];


//          pose.orientation.x = orientation_f.getX();
//          pose.orientation.y = orientation_f.getY();
//          pose.orientation.z = orientation_f.getZ();
//          pose.orientation.w = orientation_f.getW();
//          ekf_vo.pose = pose;
//          ekf_stamped.pose=pose;
        //ekf_pub.publish(ms);
//          ekf_pose_pub.publish(ekf_stamped);



       // cv::imshow("Estimated Pose2", frame2);
        cv::waitKey(1);
//        double time = tt.toc();
        //std::cout << (int)(1000.0/time) << " FPS\n";
    }
}
