#include "roomba.h"

int main(int argc,char* argv[]) {
    ros::init(argc,argv,"turtlebot_roomba");
    Roomba_bot roomba;
    ros::Rate loop(7);
    while (ros::ok()) {
        roomba.walkRoomba();
        ros::spinOnce();
        loop.sleep();
    }
    return 0;
}
