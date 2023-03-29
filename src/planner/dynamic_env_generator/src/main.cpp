#include "dynamic_env_generator.cpp"

int main(int argc, char  **argv)
{
    std::cout << "debug00" << std::endl;
    /* 初始化ROS节点 */
    ros::init(argc, argv, "Main_N");
    ros::NodeHandle main_n("~");

    DynamicEnvGenerator dynamic_env_generator;
    dynamic_env_generator.generate_dynamic_env(main_n);
    ros::spin();

    return 0;
}