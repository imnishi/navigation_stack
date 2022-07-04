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

    // -1,0,0, 0,0,1, 0,1,0,//o1//a
    // 1,0,0, 0,0,-1, 0,1,0,//o2//e
    // 0,0,1, 1,0,0, 0,1,0,//o3//f
    // 0,0,-1, -1,0,0, 0,1,0,//o4//g
    // 0.707,0,-0.707, -0.707,0,-0.707, 0,1,0,//b
    // -0.707,0,0.707, 0.707,0,0.707, 0,1,0,//d
    // 0.707,0,0.707, 0.707,0,-0.707, 0,1,0,//k
    // -0.707,0,-0.707, -0.707,0,0.707, 0,1,0,//t

    // a
    0,0,-1, -1,0,0, 0,1,0,
    // b
    0,0,1, 1,0,0, 0,1,0,
    // c
    -1,0,0, 0,0,1, 0,1,0,
    // d
    1,0,0, 0,0,-1, 0,1,0,
    // e
    0,0,1, 1,0,0, 0,1,0,
    // f
    0,0,-1, -1,0,0, 0,1,0,
    // g
    -1,0,0, 0,0,1, 0,1,0,
    // h
    1,0,0, 0,0,-1, 0,1,0,
    // i
    0,0,1, 1,0,0, 0,1,0,
    // j //not in our dictionary
    // k
    0,0,-1, -1,0,0, 0,1,0,
    // l
    0,0,-1, -1,0,0, 0,1,0,
    // m //not in our dictionary
    // n
    0,0,1, 1,0,0, 0,1,0,
    // o
    0,0,1, 1,0,0, 0,1,0,
    // p //not in our dictionary
    // q
    0,0,-1, -1,0,0, 0,1,0,
    // r
    0,0,1, 1,0,0, 0,1,0,
    // s //not in our dictionary
    // t
    0,0,-1, -1,0,0, 0,1,0,
    // u
    -0.707,0,0.707, 0.707,0,0.707, 0,1,0,
    // v
    -0.707,0,-0.707, -0.707,0,0.707, 0,1,0,
    // w
    0.707,0,-0.707, -0.707,0,-0.707, 0,1,0,
    // x
    0.707,0,0.707, 0.707,0,-0.707, 0,1,0,
};

float arucos_t_mat[60] = {

    // a
    1,7.58,0.2,
    // b
    1.2,7.58,0.2,
    // c
    4.06,6.58,0.2,
    // d
    4.06,6.38,0.2,
    // e
    3.56,0.5,0.2,
    // f
    3.36,0.5,0.2,
    // g
    0.5,1.7,0.2,
    // h
    0.5,1.5,0.2,
    // i
    1.135,4.04,0.2,
    // j
    // 1.035,4.04,0.4,
    // k
    0.935,4.04,0.2,
    // l
    2.18,6.18,0.075,
    // m
    // 2.28,6.18,0.15,
    // n
    2.38,6.18,0.075,
    // o
    3.625,4.04,0.2,
    // p
    //3.525,4.04,0.4,
    // q
    3.425,4.04,0.2,
    // r
    2.38,1.9,0.075,
    // s
    // 2.28,1.9,0.15,
    // t
    2.18,1.9,0.075,
    // u
    2.3684,4.1284,0.075,
    // v
    2.1916,4.1284,0.075,
    // w
    2.1916,3.9516,0.075,
    // x
    2.3684,3.9516,0.075
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
double camera_matrix2[]={
   1112.03, 0.0, 640.0, 0.0, 1124.28, 360.0, 0.0, 0.0, 1.0
};
double distortion_matrix[]{
    0.0777, -0.1403,
    0.0021, -0.0154,
    0.05011
};
double distortion_matrix2[]{
   0.0777, -0.1403,
   0.0021, -0.0154,
   0.05011
};

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

void get_camera_wc( cv::Vec3d& rvec,  cv::Vec3d& tvec, int id, float* op_arr,int cam_id)
{

    Eigen::Affine3f world_aruco_tf = Eigen::Affine3f::Identity(); //tf of detected aruco marker in world frame
    getTransform(world_aruco_tf, id);

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

}

void pose_estimation(cv::Mat& frame, cv::Ptr<cv::aruco::Dictionary>& aruco_dict,
                        cv::Mat& K, cv::Mat& D,
                        std::vector<cv::Vec3d>& rvecs, std::vector<cv::Vec3d>& tvecs,
                        float* final_xyzrpy,int cam_id)
{
    cv::Mat gray;
    cv::cvtColor(frame,gray, cv::COLOR_BGR2GRAY);

    // cv::boxFilter(gray, gray, -1, cv::Size(3,3));
    cv::GaussianBlur(gray, gray, cv::Size(3,3),3,3);
    // cv::bilateralFilter(frame, gray, -1, 3, 3);

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

    std::vector<int> ids;
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
        cv::aruco::drawDetectedMarkers(frame, corners, marker_ids);
        cv::aruco::estimatePoseSingleMarkers(corners, 0.15, K, D,  rvecs,  tvecs );
        std::cout<<"here3\n";
        for (int i=0;i<ids.size();i++)
        {
            cv::Vec3d rvec=rvecs[i];
            cv::Vec3d tvec=tvecs[i];//ye sahiaa rha hai:)

            get_camera_wc(rvec, tvec, ids[i], op_arr,cam2_id);
            std::cout<<"here4\n";

            camera_infos_worldFrame[i][0] = op_arr[0];
            camera_infos_worldFrame[i][1] = op_arr[1];
            camera_infos_worldFrame[i][2] = op_arr[2];
            camera_infos_worldFrame[i][3] = op_arr[3];
            camera_infos_worldFrame[i][4] = op_arr[4];
            camera_infos_worldFrame[i][5] = op_arr[5];
            cv::aruco::drawAxis(frame,K,D,rvec,tvec,0.05 );
    
        Eigen::Affine3f cTw;
        pcl::getTransformation( op_arr[0], op_arr[1], op_arr[2], roll, pitch, yaw, cTw);
        Eigen::Affine3f baseTc,baseTw,base2Tw,base2Tbase;
        // -------------------edit here-------------------------------   
        if (cam_id==0)}{
            // opp_arr=opp_arr+..78539
            // Eigen::Affine3f baseTcam
            baseTcam.matrix()<< -0.707, 0, 0.707, 0, 0, 1, 0, 0, -0.707, 0, -0.707 , -0.2898, 0, 0, 0, 1;
            baseTw= baseTc * cTw;
            

        }
        if (cam_id==2){
            // opp_arr=opp_arr-2.35619
            // Eigen::Affine3f baseTcam
            baseTcam.matrix()<< 0.707, 0, 0.707, 0, 0, 1, 0, 0, -0.707, 0, 0.707 , -0.2898, 0, 0, 0, 1;
            baseTw= baseTc * cTw;

        }
        base2Tbase.matrix()<<0,-1,0,0,  0,0,-1,0,  1,0,0,0,  0,0,0,1;
        // bTw= cTw * bTc;
        base2Tw =  basetw * base2Tbase
        for( int i=0; i<3; i++){
        op_arr[i] = base2Tw(i,3);
    }
        Eigen::Quaternionf orientation = Eigen::Quaternionf(base2Tw.rotation());
        nav_msgs::Odometry odom;
        geometry_msgs::PoseStamped pose_stamped;
        geometry_msgs::PoseWithCovariance pose_vo;
        geometry_msgs::Pose pose;
        pose.position.x = op_arr[0];
        pose.position.y = op_arr[1];
        pose.position.z = op_arr[2];
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
        odom.pose.covariance = {0.1, 0, 0, 0, 0, 0,
                                0, 0.1, 0, 0, 0, 0,
                                0, 0, 0.1, 0, 0, 0,
                                0, 0, 0, 0.1, 0, 0,
                                0, 0, 0, 0, 0.1, 0,
                                0, 0, 0, 0, 0, 0.1};
        vo_pub.publish(odom);
        //msg_required = ros::topic::waitForMessage("/robot_pose_ekf/odom_combined", create_path);
        ekf_pub.publish(msg_required);
        }
    }

   
    
int main(int argc, char** argv)
{
    ros::init(argc, argv,"aruco_cpp");

    ros::NodeHandle nh;

    // ros::Subscriber sub = nh.subscribe

    ros::AsyncSpinner aspinner(4);
    ros::Publisher ekf_pub=nh.advertise<geometry_msgs::PoseStamped>("ekf_pose", 10);
    ros::Publisher vo_pub=nh.advertise<nav_msgs::Odometry>("vo", 10);
    ros::Subscriber sub = nh.subscribe("/robot_pose_ekf/odom_combined", 10,ekf_pose_callback);
    // checking camera index
    // {
    //     cv::VideoCapture video(2);

    //     while(1)
    //     {
    //         cv::Mat frame;
    //         video >> frame;

    //         cv::imshow("temp",frame);
    //         cv::waitKey(1);

    //     }

    // }
    // return 0;

    int cam1_id = 2;
    int cam2_id = 4;

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
    // video.set(cv::CAP_PROP_FRAME_HEIGHT, 480);
    // video.set(cv::CAP_PROP_FRAME_WIDTH, 640);

    video.set(cv::CAP_PROP_FRAME_HEIGHT, 720);
    video.set(cv::CAP_PROP_FRAME_WIDTH, 1280);

    video.set(cv::CAP_PROP_FPS, 120);
    video.set(cv::CAP_PROP_FOURCC, CV_FOURCC('M','J','P','G'));

    cv::VideoCapture video2(cam2_id);
    video2.set(cv::CAP_PROP_FRAME_HEIGHT, 720);
    video2.set(cv::CAP_PROP_FRAME_WIDTH, 1280);
    video2.set(cv::CAP_PROP_FPS, 60);
    video2.set(cv::CAP_PROP_FOURCC, CV_FOURCC('M','J','P','G'));

    cv::Mat K2 = cv::Mat(3,3,CV_64FC1);
    for(int j=0;j<9;j++)
        ((double*)K2.data)[j] = camera_matrix[j];

    cv::Mat D2 = cv::Mat(1,5,CV_64FC1);
    for(int j=0;j<5;j++)
        ((double*)D.data)[j] = distortion_matrix2[j];

    cv::Mat K;
    cv::Mat D;

    cv::Mat K2;
    cv::Mat D2;

    cv::FileStorage fs1("/home/ayush/monocular/cam1/camera_parameters_1280x720.txt",cv::FileStorage::READ);
    cv::FileStorage fs2("/home/nuc/monocular/cam2/camera_parameters_1280x720.txt",cv::FileStorage::READ);

    // cv::FileStorage fs1("/home/nuc/monocular/cam1/camera_parameters_640x480.txt",cv::FileStorage::READ);
    // cv::FileStorage fs2("/home/nuc/monocular/cam2/camera_parameters_640x480.txt",cv::FileStorage::READ);

    fs1["K_left"] >> K;
    fs1["D_left"] >> D;

    fs2["K_left"] >> K2;
    fs2["D_left"] >> D2;

    // pcl::console::TicToc tt;



    aspinner.start();

    while(ros::ok())
    {
        //  tt.tic();

        cv::Mat frame;
        video >> frame;
        cv::Mat frame2;
        video2 >> frame2;

        // frame=cv2.resize(frame,(640,480),interpolation=cv2.INTER_LINEAR)

        std::vector<cv::Vec3d> rvec;
        std::vector<cv::Vec3d> tvec;

        std::vector<cv::Vec3d> rvec2;
        std::vector<cv::Vec3d> tvec2;

        float final_xyzrpy[6];
        float final2_xyzw[4];

        pose_estimation(frame, aruco_dict, K, D, rvec, tvec, final_xyzrpy, cam_id);

        pose_estimation(frame2, aruco_dict, K2, D2, rvec2, tvec2, final2_xyzw, cam2_id);
        cv::imshow("Estimated Pose", frame);

        cv::waitKey(1);
    }
}
