//在STDR上进行超声波测试
#include "ros/ros.h"
#include "sensor_msgs/Range.h"

//回调函数的消息是以boost shared_ptr指针的形式传输，意味着可以存储它而又不需要复制数据。
void sonar0_callback(const sensor_msgs::Range::ConstPtr& msg){
    ROS_INFO("sonar0 range:[%f]", msg->range);
}

int main (int argc,char **argv){
    ros::init(argc, argv, "sonar_avoidance_node");//初始化节点
    ros::NodeHandle nh;//创建节点句柄
    //通过句柄创建一个话题订阅器
    //第二个参数是队列的
    //第三个参数是回调函数
    ros::Subscriber sub = nh.subscribe("/robot0/sonar_0", 100, sonar0_callback);
    ros::spin();//进入自循环，可以尽可能快的调用消息回调函数,当ros::ok()返回false就会跳出循环关闭节点
 
    return 0;

}
