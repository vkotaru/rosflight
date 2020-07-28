#include <ros/ros.h>
#include <rosflight_msgs/Command.h>


int main(int argc, char **argv)
{
    ros::init(argc, argv, "rosflight_command_node");
    ros::NodeHandle nh;
    ROS_INFO("command node");
    
    ros::Publisher _pub_cmd = nh.advertise<rosflight_msgs::Command>("/command", 3);
 
    rosflight_msgs::Command msg;
    msg.header.stamp = ros::Time::now();
    msg.mode = rosflight_msgs::Command::MODE_ROLL_PITCH_YAWRATE_THROTTLE;
    msg.ignore = rosflight_msgs::Command::IGNORE_NONE;
    msg.x = 0.0;
    msg.y = 0.0;
    msg.z = 0.0;
    msg.F = 0.6;

    ros::Rate loop_rate(50);
    while (ros::ok()) {
        ros::spinOnce();

        msg.header.stamp = ros::Time::now();
        _pub_cmd.publish(msg);

        loop_rate.sleep();
    }


    return 0;
}
