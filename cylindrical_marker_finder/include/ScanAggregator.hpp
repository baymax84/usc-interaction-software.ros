#ifndef SCAN_AGGREGATOR_HPP
#define SCAN_AGGREGATOR_HPP

#include <boost/circular_buffer.hpp>
#include <boost/shared_ptr.hpp>

#include <vector>
#include <iomanip>
#include <algorithm>

#include "Point.hpp"
#include "Polygon.hpp"

#include "ros/ros.h"
#include "geometry_msgs/PolygonStamped.h"

//simple aggregation/foreground extraction
//all dimensions in meters unless otherwise noted in variable names
#define DEG_TO_RAD(x) x*M_PI/180.0
#define METERS_TO_INCHES(x) x*39.3700787
#define METERS_TO_FEET(x) x*3.2808399
#define FEET_TO_METERS(x) x*0.3048
#define INCHES_TO_METERS(x) x*0.0254

typedef std::vector<float> Ranges_t;
typedef std::vector<Point> FGObject;

class ScanAggregator
{
public:
    ScanAggregator(int bufferSize, float screenWFt, float screenHFt, 
                   float bgMarginFt);
    void init(const sensor_msgs::LaserScan::ConstPtr& msg);

    void addNewScan(const sensor_msgs::LaserScan::ConstPtr& msg);
    void doUpdate();


    void forceRecycle();
    bool hasValidReading() const { return m_hasValidReading; }
    Point getMarkerPtInches() const;
    Point getMarkerPt() const;

    double error(float desired, float real);

    Point getPointFromScanline(float depth, float angle);
    std::vector<float> getForegroundPts();
    int getMarkerCount() { return  m_markers.size(); }

    //For visualization
    sensor_msgs::LaserScanPtr getBGSubScan() const
    {
        return generateScanMsgFromRanges(m_avRanges);
    }
    geometry_msgs::PolygonStamped getBoundsMsg() const
    {
        return m_bounds.getPolygonMsg();
    }
    sensor_msgs::PointCloud getScreenPtsAsPointCloud() const
    {
        return pointVectorAsPointCloudMsg(m_origins);
    }
    sensor_msgs::PointCloud getMarkerPtsAsPointCloud() const
    {
        if(m_hasValidReading)
            return pointVectorAsPointCloudMsg(m_markers);
        else
            return pointVectorAsPointCloudMsg(std::vector<Point>());
    }
    bool initialized() { return m_initialized; }

private:
    boost::circular_buffer<Ranges_t> m_buffer;

    //how many scans to add before reprocessing
    int m_bufferSize;

    int m_ptsPerScan, m_scansSinceLastUpdate;
    float m_idealScreenWidth, m_idealScreenHeight, m_bgMargin;
    float m_maxAngle, m_minAngle, m_angleIncrement, m_markerRadius, 
          m_bottomHeight;
    bool m_havePrevOrigins, m_initialized, m_hasValidReading;

    //averaged laser scan
    std::vector<float> m_avRanges;
    //set of points that make up a foreground object
    std::vector<FGObject> m_foregroundObjs;
    //individual estimated points for each origin and marker
    std::vector<Point> m_origins, m_markers;
    //the bounds to use for filtering the background
    Polygon m_bounds;

    void averageBuffer();
    std::vector<float> filterBackground(const std::vector<float>& ranges);
    std::vector<FGObject> extractForegroundObjects(int maxMissing, 
                                                   float maxObjDepthVariance);
    std::vector<Point> getScreenOrigins(std::vector<Point> &objects);
    bool measuredDistanceCheck(Point a, Point b, float ideal, float tol);
    Point transformCoordinates(const Point &p) const;
    void updatePtVectors(Point p, std::vector<Point> &add, 
                         std::vector<Point> &rem);
};

typedef boost::shared_ptr<ScanAggregator> ScanAggregatorPtr;

#endif
