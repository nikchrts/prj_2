#include "ros/ros.h"
#include <message_filters/subscriber.h>
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/Point.h>



class Node{
    private:
        ros::NodeHandle n;
        ros::Subscriber data;
        geometry_msgs::Point point;
        double th_rad, velocity, radius, omega, x, y, th, vx, vy, dt;

        // compute and publish odometry
        tf::TransformBroadcaster br;
        nav_msgs::Odometry odom;
        geometry_msgs::Quaternion geom_q;
        geometry_msgs::TransformStamped geom_trans;
        ros::Publisher pub_odom;
        ros::Time current_time, last_time;

    public:
        Node(){
            x = 0;
            y = 0;
            th = 0;

            data = n.subscribe("/speedsteer", 1, &Node::Odom, this);
            pub_odom = n.advertise<nav_msgs::Odometry>("navigation/odom", 1);

            current_time = ros::Time::now();
            last_time = ros::Time::now();
        }

    void Odom(const geometry_msgs::PointStamped::ConstPtr& msg){
        point = msg->point;
        th_rad = point.x/18*3.14159/180;            // from degrees to rad
        velocity = point.y/3.6;                     // y is expressed in km/h, we need m/s
        radius = 1.765/tan(th_rad);
        omega = velocity/radius;

        Node::odometry(velocity, omega);
    }

    void odometry(double velocity, double omega){
        // compute pose parameters
        vx = velocity*cos(th);
        vy = velocity*sin(th);

        current_time = ros::Time::now();
        dt = (current_time - last_time).toSec();
        x += vx*dt;
        y += vy*dt;
        th += omega*dt;

        // create the tf transformation
        geom_trans.header.stamp = current_time;
        geom_trans.header.frame_id = "world";
        geom_trans.child_frame_id = "car";

        geom_q = tf::createQuaternionMsgFromYaw(th);
        geom_trans.transform.translation.x = x;
        geom_trans.transform.translation.y = y;
        geom_trans.transform.translation.z = 0.0;
        geom_trans.transform.rotation = geom_q;

        //send the transform
        br.sendTransform(geom_trans);

        // publish nav_msgs/Odometry topic
        odom.header.stamp = current_time;
        odom.header.frame_id = "world";

        odom.pose.pose.position.x = x;
        odom.pose.pose.position.y = y;
        odom.pose.pose.position.z = 0.0;
        odom.pose.pose.orientation = geom_q;

        odom.child_frame_id = "car";
        odom.twist.twist.linear.x = vx;
        odom.twist.twist.linear.y = vy;
        odom.twist.twist.angular.z = omega;

        pub_odom.publish(odom);

        last_time = current_time; 
    }
};   

int main(int argc, char **argv){
    ros::init(argc, argv, "ack_odom");
    Node ack_odom;
    ros::spin();
}