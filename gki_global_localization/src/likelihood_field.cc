#include "likelihood_field.h"
#include "map.h"
#include <math.h>
#include <iostream>
#include <vector>
#include <queue>
#include <algorithm>
#include <boost/foreach.hpp>

using std::cout;
using std::endl;
using std::vector;

// 这里传进来处理后的激光雷达数据，拿到定位位姿
tf::Pose LikelyHoodField::likelyHoodFieldModel(const LaserData& data) {
    const LikelyHoodFieldMap& coarsest =
        fieldMaps.back();       // 拿到最后一帧地图，分辨率最粗糙
    const int theta_steps = 2;  // 搜索步长
    // 下面这个for循环，每一个theta，都会对应一个entry，entry包含了得分信息，用一个优先队列保存得分
    for (double theta = 0; theta < 360; theta += theta_steps) {
        vector<tf::Vector3> projectedPoses = projectData(theta, data);
        // 得到以theta角度为坐标系的雷达扫描数据，且已经转为在栅格地图中的索引值
        rotatedPoses.push_back(projectedPoses);
        // 概率 = 一帧激光数据中的点数 *
        double prob = projectedPoses.size() * coarsest.getEntry(0, 0);
        size_t xt0 = 0;
        size_t xt1 = fieldMaps[0].xSize() - 1;
        size_t yt0 = 0;
        size_t yt1 = fieldMaps[0].ySize() - 1;
        /* Since the coarsest map has only 1 cell all poses have the
           same probability */
        double max_resolution = coarsest.getResolution();
        int poseindex = rotatedPoses.size() - 1;
        Entry entry(prob, xt0, xt1, yt0, yt1, max_resolution, poseindex);
        heap.push(entry);  // 用一个优先队列存储每一个theta对应的entry
    }

    Entry best = extractProbability();
    tf::Vector3 position;
    position.setX(MAP_WXGX(occupancyGrid, (int)best.xt0));
    position.setY(MAP_WYGY(occupancyGrid, (int)best.yt0));
    // Since we save the poses for each theta we can compute theta by dividing
    // the poseindex by #theta_steps
    tf::Quaternion orientation = tf::createQuaternionFromYaw(
        (double)(best.poseindex * theta_steps) / 180.00 * M_PI);
    tf::Pose result(orientation, position);

    // Clean up
    heap = std::priority_queue<Entry>();
    rotatedPoses.clear();
    return result;
}

const Entry LikelyHoodField::extractProbability() {
    Entry entry = heap.top();
    heap.pop();
    while (entry.resolution != 1) {
        if (heap.empty()) {
            ROS_ERROR(
                "Warning: Could not find matching pose for laser data."
                "Returned pose is wrong.");
            ROS_INFO(
                "Warning: Could not find matching pose for laser data."
                "Returned pose is wrong.");
            return entry;
        }
        // std::cout << "Current Heap size: " << heap.size() << std::endl;
        // std::cout << "Retrieved element with probabability " <<
        // entry.probability
        //    << " and resolution/translation " << entry.resolution << "/ x:"
        //    << entry.xt0 << "," << entry.xt1 << " y: " << entry.yt0 << ","
        //    << entry.yt1 << std::endl;
        split(entry);
        entry = heap.top();
        heap.pop();
    }
    // std::cout << "Retrieved element with probabability " << entry.probability
    //    << " and resolution/translation " << entry.resolution << "/ x:"
    //    << entry.xt0 << "," << entry.xt1 << " y: " << entry.yt0 << ","
    //    << entry.yt1 << " theta/2: " << entry.poseindex << std::endl;
    return entry;
}

void LikelyHoodField::split(Entry const& entry) {
    double diff_x = (entry.xt1 - entry.xt0) / 2;
    double diff_y = (entry.yt1 - entry.yt0) / 2;
    // Create four new entries, with one quarter of the translation (x/2,y/2)
    // s,t is the start of x,y respectively
    // Entry for x=[s,x/2] y=[t,y/2]
    computeEntry(entry.xt0, entry.xt1 - diff_x, entry.yt0, entry.yt1 - diff_y,
                 entry);

    // Entry for x=[x/2, x] y=[t,y/2]
    computeEntry(entry.xt1 - diff_x + 1, entry.xt1, entry.yt0,
                 entry.yt1 - diff_y, entry);

    // Entry for x=[s,x/2] y=[y/2, y]
    computeEntry(entry.xt0, entry.xt1 - diff_x, entry.yt1 - diff_y + 1,
                 entry.yt1, entry);

    // Entry for x=[x/2, x] y=[y/2,y ]
    computeEntry(entry.xt1 - diff_x + 1, entry.xt1, entry.yt1 - diff_y + 1,
                 entry.yt1, entry);
}

void LikelyHoodField::computeEntry(size_t xt0, size_t xt1, size_t yt0,
                                   size_t yt1, Entry const& old) {
    double prob = 0;
    int const newRes = old.resolution / 2;
    int const mapindex = log2(old.resolution) - 1;
    size_t const maxX = fieldMaps[mapindex].xSize();
    size_t const maxY = fieldMaps[mapindex].ySize();
    BOOST_FOREACH (const tf::Vector3& pose, rotatedPoses[old.poseindex]) {
        // Translate and decimate the poses
        size_t decimated_x = (pose.getX() + xt0) / newRes;
        size_t decimated_y = (pose.getY() + yt0) / newRes;
        if (decimated_x >= maxX || decimated_y >= maxY) {
            // If the translated and decimated points do not lie in the map
            // these translations are invalid and we don't have to insert a new
            // entry
            // cout << "out of range: x,y: " << pose.getX() << "," <<
            // pose.getY() << endl; cout << "with max range: x,y" << maxX << ","
            // << maxY << endl;
            return;
        }
        prob += fieldMaps[mapindex].getEntry(decimated_x, decimated_y);
    }
    Entry finer(prob, xt0, xt1, yt0, yt1, newRes, old.poseindex);
    heap.push(finer);
}

inline bool LikelyHoodField::outOfBounds(const int& x, const int& y) const {
    return (occupancyGrid->cells[MAP_INDEX(occupancyGrid, x, y)].occ_state !=
            -1);
}

// 每个theta，都将完整的激光扫描数据投影到这个theta下，得到一帧投影后的点云
vector<tf::Vector3> LikelyHoodField::projectData(double const& theta,
                                                 LaserData const& data) const {
    vector<tf::Vector3> projectedPoses;
    tf::Vector3 position;
    // 用两个宏定义函数得到原点(0，0)索引 的 坐标值
    position.setX(MAP_WXGX(occupancyGrid, 0));
    position.setY(MAP_WYGY(occupancyGrid, 0));
    /* Since we save the poses for each theta we can compute theta by dividing
       the poseindex by #theta_steps */
    tf::Quaternion orientation =
        tf::createQuaternionFromYaw(theta / 180 * M_PI);
    // tf::Pose是一个旋转矩阵
    // 最终得到一个tf坐标pose，但是这个原点不是只能表示栅格地图的(0，0)吗
    tf::Pose pose(orientation, position);
    // STL遍历器，得到处理好的激光雷达数据(ranges)vector中的每个元素(point)
    BOOST_FOREACH (tf::Vector3 point, data.ranges) {
        tf::Vector3 projectedPoint = pose * point;  // 投影到地图原点？
        // 从投影点得到栅格地图上面的索引
        projectedPoint.setX(MAP_GXWX(occupancyGrid, projectedPoint.getX()));
        projectedPoint.setY(MAP_GYWY(occupancyGrid, projectedPoint.getY()));
        projectedPoses.push_back(projectedPoint);
    }
    return projectedPoses;  // 得到theta下，完整一帧雷达扫描数据，在栅格地图中的索引编号
}

// 初始化似然域地图
void LikelyHoodField::initializeLikelyHoodFieldMap() {
    double const z_hit_denom = 2.0 * sigma_hit * sigma_hit;
    double const z_hit_mult = 1.0 / sqrt(2 * M_PI * sigma_hit);
    double const randomNoise = z_rand / max_range;
    double max_dist = occupancyGrid->max_occ_dist;

    for (int y = 0; y < fieldMaps[0].ySize(); ++y) {
        for (int x = 0; x < fieldMaps[0].xSize(); ++x) {
            int pose_x = fieldMaps[0].valueWithoutResolution(x);
            int pose_y = fieldMaps[0].valueWithoutResolution(y);
            double distToObstacle = distanceToNearestObstacle(pose_x, pose_y);
            double gaussNoise =
                z_hit * z_hit_mult *
                exp(-(distToObstacle * distToObstacle) / z_hit_denom);
            // Likelihood fields contain log probabilities, see Wikipedia for
            // further information on log probabilities
            fieldMaps[0].setEntry(-log(gaussNoise + randomNoise), x, y);
        }
    }
    // Stepwise initialization of coarser maps, until we can represent the whole
    // grid as a single point in the map
    while (fieldMaps.back().gridSize() != 1) {
        constructCoarserMap(fieldMaps.back());
    }
}

// 降低一半的分辨率来构建一个似然域地图
void LikelyHoodField::constructCoarserMap(LikelyHoodFieldMap const& source) {
    LikelyHoodFieldMap map(fieldMaps[0].xSize(), fieldMaps[0].ySize(),
                           2 * source.getResolution());
    // -1，0，1代表了障碍物的三种情况
    vector<size_t> xCoords;
    xCoords.push_back(-1);
    xCoords.push_back(0);
    xCoords.push_back(1);
    vector<size_t> yCoords;
    yCoords.push_back(-1);
    yCoords.push_back(0);
    yCoords.push_back(1);

    for (size_t y = 0; y < source.ySize(); y += 2) {
        for (size_t x = 0; x < source.xSize(); x += 2) {
            // 应该是整了个3x3的卷积核来提取特征吧
            // Convolution kernel of 3: Project a 3x3 grid around the original
            // point and compute the maximum value
            vector<double> values;
            BOOST_FOREACH (size_t const& x_offset, xCoords) {
                BOOST_FOREACH (size_t const& y_offset, yCoords) {
                    // Note that values < 0 overflow to max size_t value
                    size_t x_coord = x + x_offset;
                    size_t y_coord = y + y_offset;
                    if (x_coord < source.xSize() && y_coord < source.ySize()) {
                        values.push_back(source.getEntry(x_coord, y_coord));
                    }
                }
            }
            // Note that the max element has the lowest log probability, thus
            // we check for the smallest element
            vector<double>::iterator max =
                std::min_element(values.begin(), values.end());
            map.setEntry(*max, x / 2, y / 2);
        }
    }
    fieldMaps.push_back(map);
}

void LikelyHoodField::toPGM(std::string const& filename) {
    int i = 1;
    BOOST_FOREACH (LikelyHoodFieldMap const& fieldMap, fieldMaps) {
        fieldMap.toPGM(boost::lexical_cast<std::string>(i) + filename);
        ++i;
    }
}

inline double LikelyHoodField::distanceToNearestObstacle(int const& x,
                                                         int const& y) const {
    return occupancyGrid->cells[MAP_INDEX(occupancyGrid, x, y)].occ_dist;
}
