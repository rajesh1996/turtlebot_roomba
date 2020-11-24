/**
 * @file roomba.cpp
 * @author Rajeshwar N S
 * @copyright 2020 Rajeshwar N S
 * @brief Source file containing the various definitions like callbacks and robot movement
 */

/**
 *MIT License
 *Copyright (c) 2020 Rajeshwar N S
 *Permission is hereby granted, free of charge, to any person obtaining a copy
 *of this software and associated documentation files (the "Software"), to deal
 *in the Software without restriction, including without limitation the rights
 *to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 *copies of the Software, and to permit persons to whom the Software is
 *furnished to do so, subject to the following conditions:
 *The above copyright notice and this permission notice shall be included in all
 *copies or substantial portions of the Software.
 *THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 *IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 *FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 *AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 *LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 *OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 *SOFTWARE.
 */
#include <iostream>
#include "../include/roomba.h"

/**
 * @fn  Roomba_bot()
 * @brief Constructor definition
 *
 */
Roomba_bot::Roomba_bot() {
    // publisher to publish verlocities to robot
    pub_vel = nh.advertise < geometry_msgs::Twist > ("/cmd_vel", 1000);
    // subsriber for laserscan topic
    sub = nh.subscribe < sensor_msgs::LaserScan
    > ("/scan", 1000, &Roomba_bot::laserCallback, this);
    // Initial values
    lin_vel = 0.2;
    ang_vel = 1.0;
    msg.linear.x = 0.0;
    msg.linear.y = 0.0;
    msg.linear.z = 0.0;
    msg.angular.x = 0.0;
    msg.angular.y = 0.0;
    msg.angular.z = 0.0;
    pub_vel.publish(msg);
    obstacle = false;
}
/**
 * @fn  ~Roomba_bot()
 * @brief Destructor definition
 *
 */
Roomba_bot::~Roomba_bot() {
    // stop config
    msg.linear.x = 0.0;
    msg.linear.y = 0.0;
    msg.linear.z = 0.0;
    msg.angular.x = 0.0;
    msg.angular.y = 0.0;
    msg.angular.z = 0.0;
    pub_vel.publish(msg);
}
/**
 * @fn void laserCallback(const sensor_msgs::LaserScan::ConstPtr&)
 * @brief callback definition
 *
 * @param msg
 */
void Roomba_bot::laserCallback(const sensor_msgs::LaserScan::ConstPtr &msg) {
    // limit the lidar scan to paticular angles.
    // These can be changed accordingly
    if (msg->ranges[0] > 0.8 && msg->ranges[15] >
        0.8 && msg->ranges[345] > 0.8) {
        setObstacle(false);
    } else {
        setObstacle(true);
    }
}

/**
 * @fn bool checkObstacle()
 * @brief getter function
 *
 * @return obstacle
 */
bool Roomba_bot::checkObstacle() {
    return obstacle;
}
/**
 * @fn bool setObstacle(bool)
 * @brief setter function
 *
 * @param obs
 * @return
 */
bool Roomba_bot::setObstacle(bool obs) {
    obstacle = obs;
    return true;
}
/**
 * @fn void walkRoomba()
 * @brief method definition to implement roomba movement
 *
 */
void Roomba_bot::walkRoomba() {
    if (checkObstacle()) {
        ROS_INFO_STREAM("Obstacle present- Turning");
        // Turn by giving angular velocity and set linear vel to zero
        msg.linear.x = 0.0;
        msg.angular.z = ang_vel;
    } else {
        ROS_INFO_STREAM("Moving straight");
        // Go straight by setting a linear vel and set angular vel to zero
        msg.angular.z = 0.0;
        msg.linear.x = lin_vel;
    }
    pub_vel.publish(msg);
}
