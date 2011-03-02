#ifndef POLYGON_HPP
#define POLYGON_HPP

#include "Point.hpp"
#include "SimpleMath.hpp"
#include "LaserScanHelpers.hpp"

#include <vector>
#include <iostream>

//#ifdef USING_ROS
#include "ros/ros.h"
#include "geometry_msgs/PolygonStamped.h"
#include "geometry_msgs/Point32.h"
//#endif

//A Polygon with functions for determining if a point is inside or
//outside the bounds
class Polygon
{
public:
    Polygon() {}
    //assumes that consecutive adds are pts with an edge between them
    //and m_pts.end is connected to m_pts.begin
    void addPt(const Point& p)
    {
        m_pts.push_back(p);
    }

    //returns true if p is inside the polygon and false otherwise
    //pathological cases where p is on a vertex or edge are not
    //explicitly handled and result in undefined behavior
    bool isInside(const Point& p)
    {
        if(m_pts.size() < 2)
            return false;
        double angle = 0;
        Point p1, p2;

        for(size_t i = 0; i < m_pts.size(); ++i)
        {
            p1 = m_pts.at(i); 
            p1 -= p;
            p2 = m_pts.at((i+1)%m_pts.size());
            p2 -= p;
            angle += checkAngle(p1.x(), p1.y(), p2.x(), p2.y());
        }

        if(fabs(angle) < PI)
            return false;
        else
            return true;
    }

    double checkAngle(double x1, double y1, double x2, double y2)
    {
        double dtheta,theta1,theta2;

        theta1 = atan2(y1,x1);
        theta2 = atan2(y2,x2);
        dtheta = theta2 - theta1;
        while (dtheta > M_PI)
            dtheta -= 2*M_PI;
        while (dtheta < -M_PI)
            dtheta += 2*M_PI;

        return(dtheta);
    }

    geometry_msgs::PolygonStamped getPolygonMsg() const
    {
        geometry_msgs::PolygonStamped p;
        p.header.frame_id = "laser";
        p.header.stamp = ros::Time::now();

        for(std::vector<Point>::const_iterator i = m_pts.begin(); 
            i != m_pts.end(); ++i)
        {
            p.polygon.points.push_back(getPointAsPoint32Msg((*i)));
        }

        return p;
    }

private:
    std::vector<Point> m_pts;
};

#endif
