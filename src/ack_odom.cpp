#include "ros/ros.h"
#include <message_filters/subscriber.h>
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/Point.h>



class Node{
    private: 
        ros::NodeHandle n;
        double x, y, th;                        // odometry data to be published
        ros::Subscriber data;                   // get data from bag
        ros::Publisher pub_odom;                // publish odometry in topic
        ros::Time last_time;
        tf::TransformBroadcaster br;

    public:
        Node(){
            x = 0;
            y = 0;
            th = 0;

            data = n.subscribe("/speedsteer", 1, &Node::Odometry, this);
            pub_odom = n.advertise<nav_msgs::Odometry>("navigation/odom", 1);

            // current_time = ros::Time::now();
            last_time = ros::Time::now();
        }

    void Odometry(const geometry_msgs::PointStamped::ConstPtr& msg){
        geometry_msgs::Point temp=msg->point;
        double th_rad, velocity, omega, vx, vy, dt;
        std_msgs::Header header;
        ros::Time current_time;

        // Ackermann model
        // point = msg->point;
        th_rad = temp.x/18*3.14159/180;            // from degrees to rad
        velocity = temp.y/3.6;                     // y is expressed in km/h, we need m/s
        omega = velocity*tan(th_rad)/1.765;

        // compute pose parameters
        vx = velocity*cos(th);
        vy = velocity*sin(th);

        header = msg->header;
        current_time = header.stamp;
        dt = (current_time - last_time).toSec();
        if(abs(dt) > 1){
            last_time = header.stamp;
            dt = (header.stamp - last_time).toSec();
        }
        x += vx*dt;
        y += vy*dt;
        th += omega*dt;

        // publish nav_msgs/Odometry topic
        nav_msgs::Odometry odom;
        odom.header = header;
        odom.header.frame_id = "odom";
        odom.child_frame_id = "base_link";
        odom.pose.pose.position.x = x;
        odom.pose.pose.position.y = y;
        odom.pose.pose.position.z = 0.0;
        odom.pose.pose.orientation = tf::createQuaternionMsgFromYaw(th);
        odom.twist.twist.linear.x = vx;
        odom.twist.twist.linear.y = vy;
        odom.twist.twist.angular.z = omega;
        pub_odom.publish(odom);

        // create the tf transformation
        geometry_msgs::TransformStamped geom_trans;
        geom_trans.header.stamp = current_time;
        geom_trans.header.frame_id = "odom";
        geom_trans.child_frame_id = "base_link";
        geom_trans.transform.translation.x = x;
        geom_trans.transform.translation.y = y;
        geom_trans.transform.translation.z = 0.0;
        geom_trans.transform.rotation = tf::createQuaternionMsgFromYaw(th);
        br.sendTransform(geom_trans);

        last_time = current_time; 
    }
};   

int main(int argc, char **argv){
    ros::init(argc, argv, "ack_odom");
    Node ack_odom;
    ros::spin();
}