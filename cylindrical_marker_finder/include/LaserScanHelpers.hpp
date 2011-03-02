#ifndef LASERSCANHELPERS_HPP
#define LASERSCANHELPERS_HPP

#include "sensor_msgs/LaserScan.h"
#include "sensor_msgs/PointCloud.h"
#include "geometry_msgs/Point32.h"
#include <vector>
#include "Point.hpp"

sensor_msgs::LaserScanPtr generateScanMsgFromRanges(
    const std::vector<float> & ranges);
geometry_msgs::Point32 getPointAsPoint32Msg(Point in);
Point getFGObjectAsPoint(const std::vector<Point> &f);
sensor_msgs::PointCloud getFGObjectsAsPointCloud();
sensor_msgs::PointCloud pointVectorAsPointCloudMsg(std::vector<Point> p);

#endif
