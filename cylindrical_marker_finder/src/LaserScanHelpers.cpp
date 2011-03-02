#include "LaserScanHelpers.hpp"
#include "ros/ros.h"
#include "SimpleMath.hpp"

sensor_msgs::LaserScanPtr generateScanMsgFromRanges(const std::vector<float> & ranges)
{
    float maxAngle = M_PI/2;
    float minAngle = -M_PI/2;
    float angleIncrement = degToRad(0.5);
    unsigned int num_readings = 360;
    float laser_frequency = 75;

    ros::Time scan_time = ros::Time::now();

    //populate the LaserScan message
    sensor_msgs::LaserScanPtr scan = 
        sensor_msgs::LaserScanPtr(new sensor_msgs::LaserScan);
    scan->header.stamp = scan_time;
    scan->header.frame_id = "laser";
    scan->angle_min = minAngle;
    scan->angle_max = maxAngle;
    scan->angle_increment = angleIncrement;
    scan->time_increment = (1 / laser_frequency) / (num_readings);
    scan->range_min = 0.0;
    scan->range_max = 100.0;

    scan->set_ranges_size(num_readings);
    scan->set_intensities_size(num_readings);
    scan->ranges = ranges;

    for(unsigned int i = 0; i < num_readings; ++i){
        scan->intensities[i] = 255;
    }

    return scan;
}


geometry_msgs::Point32 getPointAsPoint32Msg(Point in)
{
    geometry_msgs::Point32 p;
    p.x = in.x();
    p.y = in.y();
    p.z = in.z();
    return p;
}

sensor_msgs::PointCloud pointVectorAsPointCloudMsg(std::vector<Point> p)
{
    sensor_msgs::PointCloud c;
    c.header.frame_id = "laser";
    c.header.stamp = ros::Time::now();
    sensor_msgs::ChannelFloat32 channel;
    channel.name = "intensity";

    for(std::vector<Point>::const_iterator i = p.begin();
        i != p.end(); ++i)
    {
        c.points.push_back(getPointAsPoint32Msg(*i));
        channel.values.push_back(1.0);
    }
    c.channels.push_back(channel);
    return c;
}

Point getFGObjectAsPoint(const std::vector<Point> &f)
{
    Point temp(0,0,0);
    for(std::vector<Point>::const_iterator j = f.begin(); j != f.end(); ++j)
    {
        temp += (*j);
    }
    temp *= 1.0f/(f.size());
    return temp;
}
