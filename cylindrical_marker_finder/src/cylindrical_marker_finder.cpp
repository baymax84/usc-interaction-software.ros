#include "ros/ros.h"
#include "sensor_msgs/LaserScan.h"
#include "sensor_msgs/PointCloud.h"
#include "geometry_msgs/Point.h"
#include "laser_geometry/laser_geometry.h"
#include "tf/transform_listener.h"
#include "tf/message_filter.h"
#include "visualization_msgs/Marker.h"
#include "std_msgs/Empty.h"

#include <cylindrical_marker_finder/Marker.h>

#include "ScanAggregator.hpp"
#include <string>

#include <cstdlib>

/**
 * Finds foreground points from a laser scan and computes coordinates in plane.
 */

ScanAggregatorPtr scanAggregator;

bool getMarkerPoint(cylindrical_marker_finder::Marker::Request  &req,
         cylindrical_marker_finder::Marker::Response &res)
{
    if(scanAggregator->hasValidReading())
    {
        //std::cout << "here" << std::endl;
        Point p = scanAggregator->getMarkerPtInches();
        res.pt.x = p.x();
        res.pt.y = p.y();
        res.pt.z = p.z();
        res.count.data = int(scanAggregator->getMarkerCount());
        ROS_INFO("sending back response: [%d, %f, %f]", 
            res.count.data, p.x(), p.y());
    }
    else
    {
        res.pt.x = -1;
        res.pt.y = -1;
        res.pt.z = -1;
        res.count.data = -1;
    }

  return true;
}


void scanCallback(const sensor_msgs::LaserScan::ConstPtr& msg, 
    ScanAggregatorPtr p)
{
    p->addNewScan(msg);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "cylindrical_marker_finder");

    float laser_frequency = 75;

    scanAggregator = ScanAggregatorPtr(new ScanAggregator(5, 11.68, 8, 1.5)); 

    ros::NodeHandle n;

    ros::ServiceServer service = n.advertiseService("get_marker_point", 
        getMarkerPoint);

    ros::Subscriber sub = n.subscribe<sensor_msgs::LaserScan>(
        "scan", 
        100, 
        boost::bind(scanCallback, _1, scanAggregator));

    ros::Publisher bgSubPub = n.advertise<sensor_msgs::LaserScan>(
        "bg_sub_scan", 100);
    ros::Publisher boundsPub = n.advertise<geometry_msgs::PolygonStamped>(
        "bg_bounds", 100);
    ros::Publisher screenOriginPub = n.advertise<sensor_msgs::PointCloud>(
        "origin_pts", 100);
    ros::Publisher markerPub = n.advertise<sensor_msgs::PointCloud>(
        "marker_pts", 100);

    ros::Rate loopRate(laser_frequency);
    while( ros::ok() )
    {
        scanAggregator->doUpdate();

        bgSubPub.publish(scanAggregator->getBGSubScan());
        boundsPub.publish(scanAggregator->getBoundsMsg());
        screenOriginPub.publish(scanAggregator->getScreenPtsAsPointCloud());
        markerPub.publish(scanAggregator->getMarkerPtsAsPointCloud());

        ros::spinOnce();
        //loopRate.sleep();
    }

    return 0;
}
