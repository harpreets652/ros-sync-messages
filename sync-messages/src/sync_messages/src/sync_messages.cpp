//
// Created by harpreetsingh on 11/27/17.
//
#include <ros/ros.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <sensor_msgs/Image.h>
#include <nav_msgs/Odometry.h>


static int counter = 0;

void imageOdometrySyncCallback(const sensor_msgs::ImageConstPtr &image, const nav_msgs::OdometryConstPtr &odometry) {
    counter ++;
    ROS_INFO_STREAM("counter: " << counter);
}


int main(int argc, char **argv) {
    ros::init(argc, argv, "sync_messages");

    ros::NodeHandle nodeHandle;

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