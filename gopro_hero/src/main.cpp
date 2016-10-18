#include "ros/ros.h"
#include "gopro_hero/gopro_hero_node.hpp"

using namespace rowboat1;

int main(int argc, char* argv[])
{
    ros::init(argc, argv, "GoProHero");
    ros::NodeHandle nh;
    GoProHeroNode* node = new GoProHeroNode(nh);

    ros::AsyncSpinner spinner(2);
    spinner.start();
    ros::waitForShutdown();
//    node->start();
    return 0;
}
