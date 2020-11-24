#include <iostream>
#include "roomba.h"

Roomba_bot::Roomba_bot() {

    pub_vel = nh.advertise <geometry_msgs::Twist>
            ("/cmd_vel", 1000);

    sub = nh.subscribe<sensor_msgs::LaserScan>("/scan",1000, &Roomba_bot::laserCallback, this);
    lin_vel = 0.2;
    ang_vel = 1.0;
    msg.linear.x=0.0;
    msg.linear.y=0.0;
    msg.linear.z=0.0;
    msg.angular.x=0.0;
    msg.angular.y=0.0;
    msg.angular.z=0.0;
    pub_vel.publish(msg);
    obstacle = false;

}

Roomba_bot::~Roomba_bot() {
    msg.linear.x=0.0;
    msg.linear.y=0.0;
    msg.linear.z=0.0;
    msg.angular.x=0.0;
    msg.angular.y=0.0;
    msg.angular.z=0.0;
    pub_vel.publish(msg);

}

void Roomba_bot::laserCallback(const sensor_msgs::LaserScan::ConstPtr& msg) {
    if (msg->ranges[0] > 0.8 && msg->ranges[15] > 0.8 && msg->ranges[345] > 0.8) {
        setObstacle(false);
    } else {
        setObstacle(true);
    }
}

bool Roomba_bot::checkObstacle() {
    return obstacle;
}

bool Roomba_bot::setObstacle(bool obs) {
    obstacle = obs;
    return true;
}

void Roomba_bot::walkRoomba() {

        if (checkObstacle()) {
            ROS_INFO_STREAM("Obstacle present- Turning");
            msg.linear.x = 0.0;
            msg.angular.z = ang_vel;
        }
        else
        {
            ROS_INFO_STREAM("Moving straight");
            msg.angular.z = 0.0;
            msg.linear.x = lin_vel;
        }
        pub_vel.publish(msg);
    }



