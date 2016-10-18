#include "ros/ros.h"
#include "gopro_hero/gopro_hero_node.hpp"

using namespace gopro_hero;

int main(int argc, char* argv[])
{
    ros::init(argc, argv, "GoProHero");
    ros::NodeHandle nh;
    GoProHeroNode* node = new GoProHeroNode(nh);

    ros::AsyncSpinner spinner(3); // 3 threads?
    spinner.start();
    ros::waitForShutdown();
    return 0;
}
