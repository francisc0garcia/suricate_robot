//
// Created by zhijie on 18.04.16.
//



#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <sensor_msgs/Image.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <visualization_msgs/Marker.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/features2d/features2d.hpp>
#include "opencv2/calib3d/calib3d.hpp"
#include <tf/transform_broadcaster.h>
#include <tf/transform_datatypes.h>
#include <std_msgs/String.h>
#include <nav_msgs/Odometry.h>
#include <opencv2/xfeatures2d.hpp>
using namespace cv;
using namespace std;
using namespace sensor_msgs;
using namespace message_filters;



vector<uchar> status;
vector<float> err;
cv_bridge::CvImagePtr cv_image,cv_image_depth;
Mat gray,gray_pre,gray_dist;
Mat depth,depth_pre,depth_filter;
vector<KeyPoint> keypoint_rgb,keypoint_rgb_pre;
Mat descriptor,descriptor_pre;
vector<Point3f> point_3d_pre,point_3d;
Mat R_vec,t_vec,R_vec_delta,t_vec_delta;
Mat world=Mat::zeros(4,1,CV_64F);
Mat Transform(4,4,CV_64F);
Mat Transform_delta(4,4,CV_64F);
Mat rotation(3,3,CV_64F);
Mat rotation_delta(3,3,CV_64F);
Mat translation=Mat::zeros(3,1,CV_64F);
Mat camera_matrix = Mat::eye(3,3,DataType<double>::type);
Mat depth_camera_matrix = Mat::eye(3,3,DataType<double>::type);
Mat distCoeffs(5,1,CV_64F);
Mat depth_distCoeffs(5,1,CV_64F);
vector<Point2f> point_pre,point;
cv::Ptr<cv::flann::IndexParams> indexParams = cv::makePtr<cv::flann::LshIndexParams>(5,10,1); // instantiate LSH index
cv::Ptr<cv::flann::SearchParams> searchParams = cv::makePtr<cv::flann::SearchParams>(50);       // instantiate flann search
cv::DescriptorMatcher * matcher = new cv::FlannBasedMatcher(indexParams, searchParams);         // instantiate FlannBased matcher
Ptr<ORB> orb = ORB::create(10000);
ros::Publisher marker_pub;
ros::Publisher safety_check_pub;
ros::Publisher visual_odometry;

void publishMarker(){
    uint32_t shape = visualization_msgs::Marker::CUBE;
    visualization_msgs::Marker marker;

    //tf::Quaternion q(-R_vec.at<double>(0)*CV_PI/180,R_vec.at<double>(2)*CV_PI/180,-R_vec.at<double>(1)*CV_PI/180);
    //y  x z
    tf::Quaternion q(-R_vec.at<double>(1)*CV_PI/180,-R_vec.at<double>(0)*CV_PI/180,-R_vec.at<double>(2)*CV_PI/180);
    marker.header.frame_id = "camera_link";
    marker.header.stamp = ros::Time::now();

    // Set the namespace and id for this marker.  This serves to create a unique ID
    // Any marker sent with the same namespace and id will overwrite the old one
    marker.ns = "basic_shapes";
    marker.id = 0;

    // Set the marker type.  Initially this is CUBE, and cycles between that and SPHERE, ARROW, and CYLINDER
    marker.type = shape;

    // Set the marker action.  Options are ADD, DELETE, and new in ROS Indigo: 3 (DELETEALL)
    marker.action = visualization_msgs::Marker::MODIFY;

    // Set the pose of the marker.  This is a full 6DOF pose relative to the frame/time specified in the header
   // marker.pose.position.x = -(float)world.at<double>(0)/100;
    //marker.pose.position.y = (float)world.at<double>(2)/100;
    //marker.pose.position.z = -(float)world.at<double>(1)/100;
    marker.pose.position.x = (float)world.at<double>(0)/100;
    marker.pose.position.y = (float)world.at<double>(1)/100;
    marker.pose.position.z = (float)world.at<double>(2)/100;
    marker.pose.orientation.x = q.getX();
    marker.pose.orientation.y = q.getY();
    marker.pose.orientation.z = q.getZ();
    marker.pose.orientation.w = q.getW();
    // Set the scale of the marker -- 1x1x1 here means 1m on a side
    marker.scale.x = 2;
    marker.scale.y = 2;
    marker.scale.z = 2;

    // Set the color -- be sure to set alpha to something non-zero!
    marker.color.r = 0.0f;
    marker.color.g = 1.0f;
    marker.color.b = 0.0f;
    marker.color.a = 1.0;

    marker_pub.publish(marker);

    nav_msgs::Odometry odom;
    geometry_msgs::Quaternion odom_quat;

    odom_quat = tf::createQuaternionMsgFromRollPitchYaw(-R_vec.at<double>(0)*CV_PI/180,-R_vec.at<double>(1)*CV_PI/180,-R_vec.at<double>(2)*CV_PI/180);
    odom.header.stamp = ros::Time::now();
    odom.header.frame_id = "camera_link";
    odom.pose.pose.position.x = (float)world.at<double>(0)/100;
    odom.pose.pose.position.y = (float)world.at<double>(1)/100;
    odom.pose.pose.position.z = (float)world.at<double>(2)/100;
    odom.pose.pose.orientation = odom_quat;
    visual_odometry.publish(odom);


    tf::TransformBroadcaster br;
    tf::Transform transform;
    transform.setOrigin( tf::Vector3((float)world.at<double>(0)/100,(float)world.at<double>(1)/100, (float)world.at<double>(2))/100 );
    transform.setRotation(q);
    br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "camera_link", "AsusXtion"));

}

void CameraPoseCompute()
{
    vector<int> inliers;
    solvePnPRansac(point_3d_pre, point, camera_matrix, noArray(),
                   R_vec_delta, t_vec_delta, false, 500, 1, 0.99, inliers, CV_EPNP);


    rotation_delta.at<double>(0,0) = cos(R_vec_delta.at<double>(0, 0))*cos(R_vec_delta.at<double>(2, 0))-sin(R_vec_delta.at<double>(0, 0))*cos(R_vec_delta.at<double>(1, 0))*sin(R_vec_delta.at<double>(2, 0));     rotation_delta.at<double>(0,1) = -cos(R_vec_delta.at<double>(0, 0))*sin(R_vec_delta.at<double>(2, 0))- sin(R_vec_delta.at<double>(0, 0))*cos(R_vec_delta.at<double>(1,0))*cos(R_vec_delta.at<double>(2, 0));    rotation_delta.at<double>(0,2) = sin(R_vec_delta.at<double>(0, 0))*sin(R_vec_delta.at<double>(1,0));
    rotation_delta.at<double>(1,0) = sin(R_vec_delta.at<double>(0, 0))*cos(R_vec_delta.at<double>(2, 0))+cos(R_vec_delta.at<double>(0, 0))*cos(R_vec_delta.at<double>(1, 0))*sin(R_vec_delta.at<double>(2, 0));     rotation_delta.at<double>(1,1) = -sin(R_vec_delta.at<double>(0, 0))*sin(R_vec_delta.at<double>(2, 0))+cos(R_vec_delta.at<double>(0, 0))*cos(R_vec_delta.at<double>(1,0))*cos(R_vec_delta.at<double>(2, 0));     rotation_delta.at<double>(1,2) = -cos(R_vec_delta.at<double>(0, 0))*sin(R_vec_delta.at<double>(1,0));
    rotation_delta.at<double>(2,0) = sin(R_vec_delta.at<double>(1, 0))*sin(R_vec_delta.at<double>(2, 0));                                                                                         rotation_delta.at<double>(2,1) = sin(R_vec_delta.at<double>(1, 0))*cos(R_vec_delta.at<double>(2, 0));                                                                                         rotation_delta.at<double>(2,2) = cos(R_vec_delta.at<double>(1, 0));


    Transform_delta.at<double>(0,0) = rotation_delta.at<double>(0,0); Transform_delta.at<double>(0,1) = rotation_delta.at<double>(0,1); Transform_delta.at<double>(0,2) = rotation_delta.at<double>(0,2); Transform_delta.at<double>(0,3) = t_vec_delta.at<double>(0);
    Transform_delta.at<double>(1,0) = rotation_delta.at<double>(1,0); Transform_delta.at<double>(1,1) = rotation_delta.at<double>(1,1); Transform_delta.at<double>(1,2) = rotation_delta.at<double>(1,2); Transform_delta.at<double>(1,3)  = t_vec_delta.at<double>(1);
    Transform_delta.at<double>(2,0) = rotation_delta.at<double>(2,0); Transform_delta.at<double>(2,1) = rotation_delta.at<double>(2,1); Transform_delta.at<double>(2,2) = rotation_delta.at<double>(2,2); Transform_delta.at<double>(2,3) =   t_vec_delta.at<double>(2);
    Transform_delta.at<double>(3,0) = 0; Transform_delta.at<double>(3,1) = 0; Transform_delta.at<double>(3,2) = 0; Transform_delta.at<double>(3,3) = 1;

    //Transform = Transform_delta*Transform;
    world = Transform_delta.inv()*world;


    //----------------------------the euler angles at current timestamp----------------------------------------
    //roll
    R_vec.at<double>(0, 0) =
            R_vec.at<double>(0, 0) + R_vec_delta.at<double>(0, 0) / CV_PI * 180;
    //pitch
    R_vec.at<double>(1, 0) =
            R_vec.at<double>(1, 0) + R_vec_delta.at<double>(1, 0) / CV_PI * 180;
    //yaw
    R_vec.at<double>(2, 0) =
            R_vec.at<double>(2, 0) + R_vec_delta.at<double>(2, 0) / CV_PI * 180;


}

void init()
{
    if (depth_pre.empty())
        depth.copyTo(depth_pre);
    if (gray_pre.empty())
    {
        keypoint_rgb_pre = keypoint_rgb;
        descriptor.copyTo(descriptor_pre);
        gray.copyTo(gray_pre);
    }
}

void update()
{
    descriptor.copyTo(descriptor_pre);
    depth.copyTo(depth_pre);
    gray.copyTo(gray_pre);
    swap(keypoint_rgb_pre, keypoint_rgb);
    point_pre.clear();
    point.clear();
    point_3d_pre.clear();
}
void display()
{
    cout << "euler angles " << R_vec << endl;
    cout << "world point " << world << endl;
}
void matching(Mat descriptor1,Mat descriptor2,DescriptorMatcher * matcher,vector<DMatch> &symMatches)
{
    std::vector<std::vector<cv::DMatch> > matches12,matches21;
    std::vector<cv::DMatch> goodmatches12,goodmatches21;
    const float ratio = 0.8; // As in Lowe's paper; can be tuned

    matcher->knnMatch(descriptor1, descriptor2,matches12, 2);

    for (int i = 0; i < matches12.size(); i++)
    {
        std::vector<cv::DMatch> temp = matches12[i];
        if ( !temp.empty() )
        if (temp[0].distance < ratio * temp[1].distance)
        {
            goodmatches12.push_back(temp[0]);
        }

    }


    matcher->knnMatch(descriptor, descriptor_pre,matches21, 2);
    for (int i = 0; i < matches21.size(); i++)
    {
        std::vector<cv::DMatch> temp = matches21[i];
        if ( !temp.empty() )
        if (temp[0].distance < ratio * temp[1].distance)
        {
            goodmatches21.push_back(temp[0]);
        }
    }


    for (vector<DMatch>::const_iterator matchIterator1= goodmatches12.begin();matchIterator1!= goodmatches12.end(); ++matchIterator1)
    {
        for (vector<DMatch>::const_iterator matchIterator2= goodmatches21.begin();matchIterator2!= goodmatches21.end();++matchIterator2)
        {
            if ((*matchIterator1).queryIdx ==(*matchIterator2).trainIdx &&(*matchIterator2).queryIdx ==(*matchIterator1).trainIdx)
            {
                symMatches.push_back(DMatch((*matchIterator1).queryIdx,(*matchIterator1).trainIdx,(*matchIterator1).distance));
                break;
            }
        }
    }
}

void get_matching_point(vector<DMatch> symMatches)
{
    float X,Y,Z,x1,y1,x2,y2;
    for(int i=0;i<symMatches.size();i++)
    {

        x1 = keypoint_rgb_pre[symMatches[i].queryIdx].pt.x;
        y1 = keypoint_rgb_pre[symMatches[i].queryIdx].pt.y;
        x2 = keypoint_rgb[symMatches[i].trainIdx].pt.x;
        y2 = keypoint_rgb[symMatches[i].trainIdx].pt.y;
        if(depth_pre.at<uint16_t>(Point((int)keypoint_rgb_pre[symMatches[i].queryIdx].pt.x,(int)keypoint_rgb_pre[symMatches[i].queryIdx].pt.y))>800 &&
           depth_pre.at<uint16_t>(Point((int)keypoint_rgb_pre[symMatches[i].queryIdx].pt.x,(int)keypoint_rgb_pre[symMatches[i].queryIdx].pt.y)) <3500 )
        if(abs(x1-x2)+abs(y1-y2) >0.55)
        {
            Z = (float)depth_pre.at<uint16_t>((int)keypoint_rgb_pre[symMatches[i].queryIdx].pt.y,(int)keypoint_rgb_pre[symMatches[i].queryIdx].pt.x);
            //Z = (float)depth_pre.at<uint16_t>((int)keypoint_rgb_pre[symMatches[i].queryIdx].pt.y,(int)keypoint_rgb_pre[symMatches[i].queryIdx].pt.x)/1000;
            X = (float)(((int)keypoint_rgb_pre[symMatches[i].queryIdx].pt.x-camera_matrix.at<double>(0,2))*Z/camera_matrix.at<double>(0,0));
            Y = (float)(((int)keypoint_rgb_pre[symMatches[i].queryIdx].pt.y-camera_matrix.at<double>(1,2))*Z/camera_matrix.at<double>(1,1));
            point_3d_pre.push_back(Point3f(X,Y,Z));
            point_pre.push_back(Point2f((int)keypoint_rgb_pre[symMatches[i].queryIdx].pt.x,(int)keypoint_rgb_pre[symMatches[i].queryIdx].pt.y));
            point.push_back(Point2f((int)keypoint_rgb[symMatches[i].trainIdx].pt.x,(int)keypoint_rgb[symMatches[i].trainIdx].pt.y));
            Z = (float)depth.at<uint16_t>((int)keypoint_rgb[symMatches[i].trainIdx].pt.y,(int)keypoint_rgb[symMatches[i].trainIdx].pt.x);
            //Z = (float)depth_pre.at<uint16_t>((int)keypoint_rgb_pre[symMatches[i].queryIdx].pt.y,(int)keypoint_rgb_pre[symMatches[i].queryIdx].pt.x)/1000;
            X = (float)(((int)keypoint_rgb[symMatches[i].trainIdx].pt.x-camera_matrix.at<double>(0,2))*Z/camera_matrix.at<double>(0,0));
            Y = (float)(((int)keypoint_rgb[symMatches[i].trainIdx].pt.y-camera_matrix.at<double>(1,2))*Z/camera_matrix.at<double>(1,1));
            point_3d.push_back(Point3f(X,Y,Z));


        }
    }
}

void callback(const ImageConstPtr& msg_rgb,const ImageConstPtr& msg_depth)
{
    cv_image = cv_bridge::toCvCopy(msg_rgb,sensor_msgs::image_encodings::BGR8);
    cv_image_depth = cv_bridge::toCvCopy(msg_depth,sensor_msgs::image_encodings::TYPE_16UC1);

    try {
        if(!cv_image->image.empty() && !cv_image_depth->image.empty()) {
            //receive image then undistor and filter
            cvtColor(cv_image->image, gray, COLOR_BGR2GRAY);
            undistort(gray,gray_dist,camera_matrix,distCoeffs);
            gray_dist.copyTo(gray);
            cv_image_depth->image.copyTo(depth);
            //undistort(depth,depth_filter,depth_camera_matrix,depth_distCoeffs);
            //depth_filter.copyTo(depth);
            GaussianBlur( gray, gray, Size(3,3), 3.0 );
            //feature detect and exact descriptor(scale and rotation invariant)
            FAST(gray,keypoint_rgb,10);
            orb->compute(gray,keypoint_rgb,descriptor);

            init();

            vector<DMatch> symMatches;
            if(descriptor.rows>80 && descriptor_pre.rows>80) {
                //matching of feature point---------------------------
                matching(descriptor_pre, descriptor, matcher, symMatches);
                //Construction of 3D-2D correspondences
                get_matching_point(symMatches);

                if(point_3d_pre.size()>30 && point.size() >30) {
                    CameraPoseCompute();
                    publishMarker();

                }
                else
                {
                //case: too close
                ROS_INFO("too close or still");

                }

            }else
            {
                //no feature detected
                ROS_INFO("no features");
            }
            update();
            display();

        }

    }
    catch (cv_bridge::Exception &e) {
        ROS_ERROR("cv_bridge exception: %s", e.what());
    }
}



int main(int argc, char** argv)
{
    camera_matrix.at<double>(0,0) = (double)267.95884148727987;
    camera_matrix.at<double>(1,1) = (double)267.93002390861574;
    //camera_matrix.at<double>(0,0) = (double)268;
    //camera_matrix.at<double>(1,1) = (double)268;
    camera_matrix.at<double>(0,2) = (double)157.70814552671587;
    camera_matrix.at<double>(1,2) = (double)112.35024605027004;
    //camera_matrix.at<double>(0,2) = (double)160;
    //camera_matrix.at<double>(1,2) = (double)120;

    distCoeffs.at<double>(0,0) = 0.02430359101358061;
    distCoeffs.at<double>(1,0) = -0.08516494676382694;
    distCoeffs.at<double>(2,0) =  0.0005722830900508588;
    distCoeffs.at<double>(3,0) = -0.0010368589280136632;
    distCoeffs.at<double>(4,0) = 0;

    depth_camera_matrix.at<double>(0,0) = (double)289.02022057482;
    depth_camera_matrix.at<double>(1,1) = (double)289.05935360987354;
    depth_camera_matrix.at<double>(0,2) = (double)159.1245933384702;
    depth_camera_matrix.at<double>(1,2) = (double)127.73325776289624;

    depth_distCoeffs.at<double>(0,0) = -0.033368808072722446;
    depth_distCoeffs.at<double>(1,0) = 0.024327517933491673;
    depth_distCoeffs.at<double>(2,0) =  -0.00023554960510656071;
    depth_distCoeffs.at<double>(3,0) = -0.0010466772257595512;
    depth_distCoeffs.at<double>(4,0) = 0;

    Transform.at<double>(0,0) = 1; Transform.at<double>(0,1) = 0; Transform.at<double>(0,2) = 0; Transform.at<double>(0,3) = 0;
    Transform.at<double>(1,0) = 0; Transform.at<double>(1,1) = 1; Transform.at<double>(1,2) = 0; Transform.at<double>(1,3) = 0;
    Transform.at<double>(2,0) = 0; Transform.at<double>(2,1) = 0; Transform.at<double>(2,2) = 1; Transform.at<double>(2,3) = 0;
    Transform.at<double>(3,0) = 0; Transform.at<double>(3,1) = 0; Transform.at<double>(3,2) = 0; Transform.at<double>(3,3) = 1;

    world.at<double>(0)=0;
    world.at<double>(1)=0;
    world.at<double>(2)=0;
    world.at<double>(3)=1;

    R_vec = Mat::zeros(3,1,CV_64F);
    t_vec = Mat::zeros(3,1,CV_64F);
    R_vec_delta = Mat::zeros(3,1,CV_64F);
    t_vec_delta = Mat::zeros(3,1,CV_64F);

    ros::init(argc, argv, "vision_node");

    ros::NodeHandle nh;
    message_filters::Subscriber<Image> image1_sub(nh, "/camera/rgb/image_raw", 1);
    message_filters::Subscriber<Image> image2_sub(nh, "/camera/depth/image_raw", 1);
    marker_pub = nh.advertise<visualization_msgs::Marker>("/robot/visualization_marker", 1);
    //safety_check_pub = nh.advertise<std_msgs::String>("/robot/vo_saftey", 1);
    visual_odometry =  nh.advertise<nav_msgs::Odometry>("/robot/camera/visual_odometry",1);
    //safety_check_pub = nh.advertise<std_msgs::String>("robot_visual_odometry_message", 1);
    typedef sync_policies::ApproximateTime<Image, Image> MySyncPolicy;
    // ApproximateTime takes a queue size as its constructor argument, hence MySyncPolicy(10)
    Synchronizer<MySyncPolicy> sync(MySyncPolicy(1), image1_sub, image2_sub);
    sync.registerCallback(boost::bind(&callback, _1, _2));
    ros::spin();

    return 0;
}
