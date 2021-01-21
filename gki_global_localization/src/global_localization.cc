#include "global_localization.h"
#include "MapDataMissingException.h"

void GlobalLocalization::processMap(const nav_msgs::OccupancyGrid& map_msg) {
    if (map != NULL) {
        // 检查当前的地图和传进来的地图是不是一致的
        bool size_changed = false;
        size_changed |= map->size_x == map_msg.info.width;
        size_changed |= map->size_y == map_msg.info.height;
        size_changed |= map->scale == map_msg.info.resolution;
        size_changed |= map->origin_x == map_msg.info.origin.position.x + (map->size_x / 2) * map->scale;
        size_changed |= map->origin_y == map_msg.info.origin.position.y + (map->size_y / 2) * map->scale;
        // 处理地图的过程中保证地图不发生变化
        if (size_changed) {
            free(map->cells);
            free(map);
            map = NULL;
        }
    }
    // 把传进来的地图数据转成定义的地图格式
    if (map == NULL) {
        map = (map_t*)malloc(sizeof(map_t));

        // Assume we start at (0, 0)
        map->origin_x = 0;
        map->origin_y = 0;

        // Make the size odd
        map->size_x = 0;
        map->size_y = 0;
        map->scale = 0;

        // Allocate storage for main map
        map->cells = (map_cell_t*)NULL;

        // ROS中的断言
        ROS_ASSERT(map);

        map->size_x = map_msg.info.width;
        map->size_y = map_msg.info.height;
        map->scale = map_msg.info.resolution;
        map->origin_x = map_msg.info.origin.position.x + (map->size_x / 2) * map->scale;
        map->origin_y = map_msg.info.origin.position.y + (map->size_y / 2) * map->scale;

        // Convert to amcl map format
        map->cells = (map_cell_t*)malloc((sizeof(map_cell_t)) * map->size_x * map->size_y);
        ROS_ASSERT(map->cells);
    }
    // 和amcl源码中的地图构建形式一样的
    for (int i = 0; i < map->size_x * map->size_y; ++i) {
        if (map_msg.data[i] == 0) {
            map->cells[i].occ_state = -1;  // free
        } else if (map_msg.data[i] == 100) {
            map->cells[i].occ_state = +1;  // occ
        } else {
            map->cells[i].occ_state = 0;  // unknown
        }
    }

    double max_occ_dist = 2.0;             // 这个参数是amcl似然域模型中的
    map_update_cspace(map, max_occ_dist);  // 处理了map数据，这里参考amcl源码

    ROS_INFO("Map received. Creating coarser likelyhood maps.");
    if (field != NULL) {
        delete field;
    }

    /**
     * @description:生成似然域地图 
     * @param {*}
     * @return {*}
     */
    field = new LikelyHoodField(map, sigma_hit, z_rand, z_hit, laser_max_range);
    field->initializeLikelyHoodFieldMap();
    map_frame_id = map_msg.header.frame_id;
    map_stamp = map_msg.header.stamp;
    ROS_INFO("Map data processed.");
}

void GlobalLocalization::processLaserScan(const sensor_msgs::LaserScan::ConstPtr& scan) {
    ROS_INFO("Laser scan received.");
    try {
        tf::Stamped<tf::Pose> localizedPose = localize(scan);
        ROS_INFO("Global pose is: %.3f %.3f %.3f",
                 localizedPose.getOrigin().x(),
                 localizedPose.getOrigin().y(),
                 tf::getYaw(localizedPose.getRotation()));
        geometry_msgs::PoseStamped poseMsg;
        tf::poseStampedTFToMsg(localizedPose, poseMsg);
        pose_pub.publish(poseMsg);
    } catch (...) {
    }
}

// 下面是从scan得到定位的过程！
tf::Stamped<tf::Pose> GlobalLocalization::localize(const sensor_msgs::LaserScan::ConstPtr& scan) {
    if (field == NULL) {
        ROS_INFO_THROTTLE(1.0, "Waiting for map data...");
        throw MapDataMissingException("No map data available!");
    }
    ROS_INFO("likelyhood grid initialized.");
    // 把激光数据传进来转成laserData中的vector数组，其实就是xy坐标值
    data = LaserData(scan, laser_max_range);
    // 下面的函数是定位的最主要函数，还统计了定位的时间
    ros::Time start = ros::Time::now();
    // 重点看likelyHoodFieldModel(data)
    tf::Stamped<tf::Pose> stampedPose(field->likelyHoodFieldModel(data), scan->header.stamp, map_frame_id);
    double time_elapsed = (ros::Time::now() - start).toSec();
    ROS_INFO("localization took %.3f seconds", time_elapsed);
    return stampedPose;
}
