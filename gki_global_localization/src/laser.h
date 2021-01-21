#ifndef LASER_H
#define LASER_H

#include <sensor_msgs/LaserScan.h>
#include <tf/transform_listener.h>
#include <math.h>
#include <vector>

class LaserData {
   public:
    LaserData() : time(0), frame_id(""), range_min(0.0), range_max(0.0) {}

    LaserData(const sensor_msgs::LaserScan::ConstPtr& scan, double max_range)
        : time(scan->header.stamp), range_min(scan->range_min) {
        // Apply range max thresholds if the user supplied them
        // 找到激光测距仪器的最大值
        if (max_range > 0.0) {
            range_max =
                std::min(static_cast<double>(scan->range_max), max_range);
        } else {
            range_max = scan->range_max;
        }

        for (size_t i = 0; i < scan->ranges.size(); ++i) {
            // Readings out of range are ignored
            // We also ignore max range readings
            if (scan->ranges[i] < range_min || scan->ranges[i] >= range_max) {
                continue;
            }
            // 把scan数据（极坐标）转成了直角坐标
            // @TODO:如果实时性很差的话，cos和sin其实可以计算好，因为增加的角度是一样的，每次只要查表就好了
            double angle = scan->angle_min + i * scan->angle_increment;
            double x = cos(angle) * scan->ranges[i];
            double y = sin(angle) * scan->ranges[i];
            tf::Vector3 range(x, y, 0);
            ranges.push_back(range);  // 最后传到ranges这个vector中
        }
    }

    ros::Time time;
    std::string frame_id;
    double range_min;
    double range_max;
    std::vector<tf::Vector3> ranges;  // 处理好的激光雷达数据
};

#endif /* LASER_H */
