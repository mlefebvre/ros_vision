#include <iostream>
#include <ros/ros.h>
#include <std_srvs/Empty.h>
#include <pcl/point_types.h>
#include <pcl/io/io.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/io/pcd_io.h>
#include <pcl_ros/transforms.h>
#include <tf/transform_listener.h>
#include <pcl/io/openni_grabber.h>
#include <pcl/io/openni_camera/openni_image.h>
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>

using namespace std;
using namespace cv;

typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;

string savePath, targetFrame;
tf::TransformListener* tfListener;
PointCloud::Ptr currentCloud;
ros::Time currentPointCloudTime;
ros::Publisher pub;
ros::Publisher imgpub;

void cloudCallback (const PointCloud::ConstPtr &cloud)
{
    currentPointCloudTime = ros::Time::now();
    pcl::copyPointCloud(*cloud, *currentCloud);
    if ( pub.getNumSubscribers() > 0 )
    {
        sensor_msgs::PointCloud2 msg;
        pcl::toROSMsg(*cloud, msg);
        msg.header.stamp = ros::Time::now();
        pub.publish(msg);
    }
}

void imageCallback (const boost::shared_ptr<openni_wrapper::Image> &image)
{
    unsigned char *rgb_buffer = new unsigned char[image->getWidth() * image->getHeight() * 3];
    image->fillRGB( image->getWidth(), image->getHeight(), rgb_buffer);
    Mat cvimage(Size(image->getWidth(), image->getHeight()), CV_8UC3, rgb_buffer, Mat::AUTO_STEP);
    Mat bgrimage;
    cvtColor(cvimage, bgrimage, CV_BGR2RGB);  
    cv_bridge::CvImage msg;
    msg.encoding = sensor_msgs::image_encodings::TYPE_8UC3;
    msg.header.stamp = ros::Time::now();
    msg.header.frame_id = "openni_rgb_optical_frame";
    msg.image    = bgrimage;
    imgpub.publish(msg.toImageMsg());
    delete(rgb_buffer);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "openni_node");
    ros::NodeHandle nh;
    ros::NodeHandle n("~");

    // Params
    nh.param<std::string>("target_frame", targetFrame, "/map");
        
    // Openni
    pcl::Grabber* interface = new pcl::OpenNIGrabber();
    boost::function<void (const PointCloud::ConstPtr&)> f = boost::bind (cloudCallback, _1);
    boost::function<void (const boost::shared_ptr<openni_wrapper::Image>&)> fimg = boost::bind (imageCallback, _1); 
    interface->registerCallback (f);
    interface->registerCallback (fimg);
    currentCloud = PointCloud::Ptr(new PointCloud);
    interface->start ();

    // Publisher
    pub = n.advertise<sensor_msgs::PointCloud2>("point_cloud", 15);    
    imgpub = n.advertise<sensor_msgs::Image>("image", 15);
    
    // TF
    tfListener = new tf::TransformListener;
    ros::spin();
    
    interface->stop();
}