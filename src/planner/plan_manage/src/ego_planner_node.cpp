#include <ros/ros.h>
#include <visualization_msgs/Marker.h>

#include <plan_manage/ego_replan_fsm.h>

using namespace ego_planner;

int main(int argc, char **argv)
{
    // launch 文件 name > 命令行 __name:= > ros::init 默认名
    ros::init(argc, argv, "ego_planner_node");
    ros::NodeHandle nh("~");

    EGOReplanFSM rebo_replan;

    rebo_replan.init(nh);

    ros::Duration(1.0).sleep();
    ros::spin();

    return 0;
}
