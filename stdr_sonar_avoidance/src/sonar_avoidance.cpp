// Description:在STDR上进行超声波避障策略仿真
#include "geometry_msgs/Twist.h"
#include "ros/ros.h"
#include "sensor_msgs/Range.h"

#define setbit(x, y) x |= (1 << y)
#define clrbit(x, y) x &= ~(1 << y)

// low three bit as sonar warn flag
//           left  font right
// x x x x  x  0    0    0
#define STATUS_A 0x04  // v x x
#define STATUS_B 0x02  // x v x
#define STATUS_C 0x01  // x x v
#define STATUS_D 0x07  // v v v
#define STATUS_E 0x06  // v v x
#define STATUS_F 0x03  // x v v
#define STATUS_G 0x05  // v x v

// global variable
geometry_msgs::Twist twist_cmd;//初始化消息
ros::Publisher twist_pub;//创建publisher

const double warn_range = 0.5;  // warn check distance

double default_period_hz = 10;  // hz
double default_linear_x = 0.5;  // (m/s)
double default_yaw_rate = 0.5;  // rad/s

double range_array[3];  // 0存储左边，1存中间，2存右边

void sonar0_callback(const sensor_msgs::Range::ConstPtr& msg) {
    ROS_INFO("front Sonar0 range:[%f]", msg->range);
    range_array[1] = msg->range;
}

void sonar1_callback(const sensor_msgs::Range::ConstPtr& msg) {
    ROS_INFO("left Sonar1 range:[%f]", msg->range);
    range_array[0] = msg->range;
}

void sonar2_callback(const sensor_msgs::Range::ConstPtr& msg) {
    ROS_INFO("right Sonar2 range:[%f]", msg->range);
    range_array[2] = msg->range;
}

void publishTwistCmd(double linear_x, double angular_z) {
    twist_cmd.linear.x = linear_x;
    twist_cmd.linear.y = 0.0;
    twist_cmd.linear.z = 0.0;

    twist_cmd.angular.x = 0.0;
    twist_cmd.angular.y = 0.0;
    twist_cmd.angular.z = angular_z;

    twist_pub.publish(twist_cmd);
}

void checkSonarRange(double sonar_l, double sonar_f, double sonar_r) {
    unsigned char flag = 0;

    if (sonar_l < warn_range) {
        setbit(flag, 2);
    } else {
        clrbit(flag, 2);
    }

    if (sonar_f < warn_range) {
        setbit(flag, 1);
    } else {
        clrbit(flag, 1);
    }

    if (sonar_r < warn_range) {
        setbit(flag, 0);
    } else {
        clrbit(flag, 0);
    }

    ROS_INFO("CheckSonarRange get status:0x%x", flag);
    switch (flag) {
        case STATUS_A:  // turn right 0x04
            ROS_WARN("left warn,turn right");
            publishTwistCmd(0, -default_yaw_rate);
            break;

        case STATUS_B:  // 0x02
            ROS_WARN(
                "front warn, left and right ok, compare left and right value "
                "to turn");
            if (sonar_l > sonar_r) {
                ROS_WARN("turn left");
                publishTwistCmd(0, default_yaw_rate);
            } else {
                ROS_WARN("turn right");
                publishTwistCmd(0, -default_yaw_rate);
            }
            break;

        case STATUS_C:  // turn left
            ROS_WARN("left ok, front ok, right warn, turn left");
            publishTwistCmd(0, default_yaw_rate);
            break;

        case STATUS_D:
            ROS_WARN("left,front,right all warn, turn back");
            publishTwistCmd(0, 10 * default_yaw_rate);
            break;

        case STATUS_E:
            ROS_WARN("left warn, front warn, right ok, turn right");
            publishTwistCmd(0, (-default_yaw_rate * 2));
            break;

        case STATUS_F:
            ROS_WARN("left ok, front warn, right warn, turn left");
            publishTwistCmd(0, (default_yaw_rate * 2));
            break;

        case STATUS_G:
            ROS_WARN("left and right warn, front ok, speed up");
            publishTwistCmd(2 * default_linear_x, 0);
            break;

        default:  // go forward straight line
            publishTwistCmd(default_linear_x, 0);
            break;
    }
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "sonar_avoidance_node");
    ros::NodeHandle handle;
    ros::Rate loop_rate = default_period_hz;

    ros::Subscriber sub_sonar0 =
        handle.subscribe("/robot0/sonar_0", 100, sonar0_callback);
    ros::Subscriber sub_sonar1 =
        handle.subscribe("/robot0/sonar_1", 100, sonar1_callback);
    ros::Subscriber sub_sonar2 =
        handle.subscribe("/robot0/sonar_2", 100, sonar2_callback);

    twist_pub = handle.advertise<geometry_msgs::Twist>("/robot0/cmd_vel", 10);

    while (ros::ok()) {
        checkSonarRange(range_array[0], range_array[1], range_array[2]);

        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}