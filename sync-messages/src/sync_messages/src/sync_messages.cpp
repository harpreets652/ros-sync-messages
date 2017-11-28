//
// Created by harpreetsingh on 11/27/17.
//
#include <ros/ros.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <sensor_msgs/Image.h>
#include <nav_msgs/Odometry.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/highgui/highgui.hpp>
#include <fstream>


static int counter = 0;

void imageOdometrySyncCallback(const sensor_msgs::ImageConstPtr &imageMessage,
                               const nav_msgs::OdometryConstPtr &odometryMessage) {
    cv_bridge::CvImagePtr cvImagePtr;
    try {
        cvImagePtr = cv_bridge::toCvCopy(imageMessage, sensor_msgs::image_encodings::BGR8);
    }
    catch (cv_bridge::Exception &e) {
        ROS_ERROR_STREAM("cv_bridge exception:" << e.what());
        return;
    }

    //todo: output directory should be an arg to this node, set in the ros master properties node, and read in the callback
    //for now, just run this node in the directory where messages should be stored
    int frameSeq = imageMessage->header.seq;
    auto numSeconds = imageMessage->header.stamp.sec;
    auto numNanSec = imageMessage->header.stamp.nsec;

    std::stringstream imageIdentifierStream;
    imageIdentifierStream << frameSeq << "_" << numSeconds << "." << numNanSec;
    std::string imageIdentifier = imageIdentifierStream.str();

    //write image
    ROS_ASSERT(cv::imwrite(imageIdentifier + ".jpg", cvImagePtr->image));

    //write odometry message
    std::ofstream ofs(imageIdentifier + ".txt");
    ofs << "odometry.seq,odometry.stamp,pose.position.x,pose.position.y,pose.position.z,"
            "pose.orientation.x,pose.orientation.y,pose.orientation.z,pose.orientation.w\n";
    ofs << odometryMessage->header.seq << ","
        << odometryMessage->header.stamp << ","
        << odometryMessage->pose.pose.position.x << ","
        << odometryMessage->pose.pose.position.y << ","
        << odometryMessage->pose.pose.position.z << ","
        << odometryMessage->pose.pose.orientation.x << ","
        << odometryMessage->pose.pose.orientation.y << ","
        << odometryMessage->pose.pose.orientation.z << ","
        << odometryMessage->pose.pose.orientation.w;

    ofs.close();

    counter++;
    ROS_INFO_STREAM("counter: " << counter << " | " << imageIdentifier);
}


int main(int argc, char **argv) {
    ros::init(argc, argv, "sync_messages");

    ros::NodeHandle nodeHandle;

    //todo: topics should be set from args to this node...hard coded for now
    message_filters::Subscriber<sensor_msgs::Image> imageSubscriber(nodeHandle, "cam0/image_raw", 1);
    message_filters::Subscriber<nav_msgs::Odometry> odometrySubscriber(nodeHandle, "msf_core/odometry", 1);

    typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, nav_msgs::Odometry> syncPolicy;
    message_filters::Synchronizer<syncPolicy> sync(syncPolicy(10), imageSubscriber, odometrySubscriber);

    sync.registerCallback(boost::bind(&imageOdometrySyncCallback, _1, _2));

    while (ros::ok()) {
        ros::spin();
    }

    return EXIT_SUCCESS;
}