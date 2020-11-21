#include <iostream>
#include <roomba.cpp>

Roomba_bot::Roomba_bot() {
    lin_vel = 0.9;
    ang_vel = 1.0;
    msg.linear.x=0.0;
    msg.linear.y=0.0;
    msg.linear.z=0.0;
    msg.angular.x=0.0;
    msg.angular.y=0.0;
    msg.angular.z=0.0;
}

Roomba_bot::~Roomba_bot() {
    msg.linear.x=0.0;
    msg.linear.y=0.0;
    msg.linear.z=0.0;
    msg.angular.x=0.0;
    msg.angular.y=0.0;
    msg.angular.z=0.0;

}

void Roomba_bot::laserCallback(const sensor_msgs::LaserScan::ConstPtr& msg) {
    for(auto i=0;i<msg->ranges.size();i++)
    {
        if (msg->ranges[i]<0.9) {
            obstacle = true;
            return;
        }

    }
    obstacle = false;
}

bool Roomba_bot::checkObstacle() {
    return obstacle;
}


