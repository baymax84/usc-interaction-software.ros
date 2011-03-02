#include "ScanAggregator.hpp"

#include <vector>
#include <map>
#include <iomanip>
#include <algorithm>
#include <numeric>
#include <functional>
#include <iostream>
#include <cassert>
#include <unistd.h>

#include "Point.hpp"
#include "Polygon.hpp"
#include "LaserScanHelpers.hpp"

#define DEG_TO_RAD(x) x*M_PI/180.0
#define METERS_TO_INCHES(x) x*39.3700787
#define METERS_TO_FEET(x) x*3.2808399
#define FEET_TO_METERS(x) x*0.3048
#define INCHES_TO_METERS(x) x*0.0254

ScanAggregator::ScanAggregator(int bufferSize, float screenWFt, float screenHFt, float bgMarginFt)
{
    m_initialized = false;
    m_havePrevOrigins = false;
    m_hasValidReading = false;
    m_bufferSize = bufferSize;
    m_scansSinceLastUpdate = 0;

    m_idealScreenWidth = FEET_TO_METERS(screenWFt);
    m_idealScreenHeight = FEET_TO_METERS(screenHFt);
    m_bgMargin = FEET_TO_METERS(bgMarginFt);

    m_markerRadius = INCHES_TO_METERS(1.5); //pringles can
    m_bottomHeight = INCHES_TO_METERS(5.5);  //without wheels
    m_buffer = boost::circular_buffer<Ranges_t>(bufferSize); 

    //setup bounds with margin feet around screen size
    //note x and y's are flipped for display purposes
    //ie. these are not in the deixis coordinate system
    double y = m_idealScreenWidth/2.0+m_bgMargin;
    double x = m_idealScreenHeight+m_bgMargin;
    m_bounds.addPt(Point(0, 0, 0));
    m_bounds.addPt(Point(0, y, 0));
    m_bounds.addPt(Point(x, y, 0));
    m_bounds.addPt(Point(x, -y, 0));
    m_bounds.addPt(Point(0, -y, 0));
}

void ScanAggregator::init(const sensor_msgs::LaserScan::ConstPtr& msg) 
{
    m_maxAngle = msg->angle_max;
    m_minAngle = msg->angle_min;
    m_angleIncrement = msg->angle_increment;
    m_ptsPerScan = (m_maxAngle-m_minAngle)/m_angleIncrement;
    m_initialized = true;
}

void ScanAggregator::addNewScan(const sensor_msgs::LaserScan::ConstPtr& msg) 
{
    if(!m_initialized)
        init(msg);
    m_buffer.push_back(filterBackground(msg->ranges));
    //m_buffer.push_back(msg->ranges);
    m_scansSinceLastUpdate++;
}

void ScanAggregator::averageBuffer()
{
    m_avRanges = std::vector<float>(m_ptsPerScan, 0.0);
    std::vector<int> counts(m_ptsPerScan, 0);

    for(boost::circular_buffer<std::vector<float> >::const_iterator i = 
        m_buffer.begin(); i != m_buffer.end(); ++i)
    {
        for(int j = 0; j < m_ptsPerScan; ++j)
        {
            if((*i)[j] != -1)
            {
                m_avRanges[j] += (*i)[j];
                counts[j] += 1;
            }
        }
    }

    for(int j = 0; j < m_ptsPerScan; ++j)
    {
        if(counts[j] == 0)
            m_avRanges[j] = -1;
        else
            m_avRanges[j] = m_avRanges[j] / counts[j];
    }
}

//TODO: this already converts scanlines to points, which we repeat later
//computes a rough bounding box based on idealScreenWidth and height and 
//assumes the laser is (roughly) in the middle of ScreenWidth looking upwards
std::vector<float> ScanAggregator::filterBackground(
    const std::vector<float>& ranges)
{
    float curAngle = m_minAngle;
    std::vector<float> retVal;
    for(std::vector<float>::const_iterator i = ranges.begin(); 
        i != ranges.end(); ++i, curAngle += m_angleIncrement)
    {
        Point p = getPointFromScanline((*i), curAngle);
        if(!m_bounds.isInside(p))
            retVal.push_back(-1);
        else
            retVal.push_back((*i));
    }
    return retVal;
}

double ScanAggregator::error(float desired, float real)
{
    return fabs(desired-real);
}

//force the buffer to refill before the next update
void ScanAggregator::forceRecycle()
{
    m_scansSinceLastUpdate = 0;
}

//called repeatedly in main to update state
void ScanAggregator::doUpdate()
{
    if(m_scansSinceLastUpdate < m_bufferSize)
        return;

    //compute the average of all scans in the buffer ignoring background pts
    averageBuffer();
    //group foreground points by checking missing scanlines and depth changes
    m_foregroundObjs = extractForegroundObjects(2, FEET_TO_METERS(4));

    //std::cout << "found " << m_foregroundObjs.size() 
    //          << " fg objects" << std::endl;

    m_markers = std::vector<Point>(m_foregroundObjs.size());
    //Compute an average point for each clustered foreground object
    std::transform(m_foregroundObjs.begin(), m_foregroundObjs.end(), 
        m_markers.begin(), getFGObjectAsPoint);

    //TODO: something cooler like EM using the bounds specified would be better
    //but isn't strictly necessary for our simple usage case
    //find the expected screen origin markers and move them from m_markers to
    //m_origins
    m_origins = getScreenOrigins(m_markers);
   // std::cout << "found " << m_origins.size() 
              //<< " origins" << std::endl;

    if( m_origins.size() == 2 && m_markers.size() >= 1 )
    {
        m_hasValidReading = true;
        Point p = getMarkerPtInches();
        std::cout << "marker: " << p << " count: " << getMarkerCount() << std::endl;
    }

    m_scansSinceLastUpdate = 0;
}

//moves a vector element from one to the other
void ScanAggregator::updatePtVectors(Point p, std::vector<Point> &add, 
                                     std::vector<Point> &rem)
{
    add.push_back(p);
    std::vector<Point>::iterator i = std::remove(rem.begin(), rem.end(), p);
    rem.erase(i);
}

//returned in left->right order
//assumptions: 
// -we always see at least one origin (occlusion check assumes that
//  the point with less error is more accurate)
// -to get the first measurement we require seeing both with error < 1%
// -returns empty vector until it above condition is met
std::vector<Point> ScanAggregator::getScreenOrigins(std::vector<Point> &objects)
{
    std::vector<Point> origins;

    //finding origins for first time and     
    if(!m_havePrevOrigins)
    {
        //didn't get 2 as expected, so return
        if(objects.size() < 2)
        {
            m_havePrevOrigins = false;
        }
        //got 2 and didn't have prior knowledge so assume these are
        else
        {
            if(!measuredDistanceCheck(objects.front(), objects.back(), 
                                     m_idealScreenWidth, 5))
            {
              //std::cerr << "Measured screen origins are off by more than 1% "
              //         << " of the ideal screen width and may be incorrect!"
              //         << std::endl;
                m_havePrevOrigins = false;
            }
            else
            {
                updatePtVectors(objects.front(), origins, objects);
                updatePtVectors(objects.back(), origins, objects);
                m_havePrevOrigins = true;
            }
        }
    }
    //had origins before check for occlusion
    else
    {
        //choose the first and last since they should be the legs
        if(objects.size() > 2)
        {
//            if(!measuredDistanceCheck(objects.front(), objects.back(), 
//                                     m_idealScreenWidth, 1))
//               std::cerr << "Measured screen origins are off by more than 5% "
//                          << " of the screen width and may be incorrect!"
//                          << std::endl;
            updatePtVectors(objects.front(), origins, objects);
            updatePtVectors(objects.back(), origins, objects);
            m_havePrevOrigins = true;
        }
        //found one or no objects
        else if(objects.size() < 2)
        {
//            std::cerr << "Found " << objects.size() << " objects! Expected "
//                      << "at least 2!" << std::endl;
            m_havePrevOrigins = false;
        }
        //have 2 origins check for occlusion and compute missing pt if necessary
        else
        {
            if(measuredDistanceCheck(objects.front(), objects.back(), 
                                     m_idealScreenWidth, 5))
            {
                updatePtVectors(objects.front(), origins, objects);
                updatePtVectors(objects.back(), origins, objects);
                m_havePrevOrigins = true;
            }
            else  //outside tolerance figure out if one is occluded
            {
                //left is wrong
               //std::cerr << "Left origin occluded adjusting..." << std::endl;
                if(distancePointPoint(m_origins[0], objects.front()) >
                   distancePointPoint(m_origins[1], objects.back()))
                {
                    updatePtVectors(objects.back(), origins, objects);
                    Point left(objects.back());
                    left.m_P[1] = left.m_P[1] - m_idealScreenWidth;
                    origins.push_back(left);
                }
                else //right is wrong
                {
//                    std::cerr << "Right origin occluded adjusting..." 
//                              << std::endl;
                    updatePtVectors(objects.front(), origins, objects);
                    Point right(objects.back());
                    right.m_P[1] = right.m_P[1] + m_idealScreenWidth;
                    origins.push_back(right);
                }
                m_havePrevOrigins = true;
            }
        }
    }

    return origins;;
}

//checks if ||a-b|| is within tol% of ideal 
bool ScanAggregator::measuredDistanceCheck(Point a, Point b, float ideal,
                                           float tol)
{        
    return(fabs(distancePointPoint(a, b) - m_idealScreenWidth) 
           < 0.01*tol*m_idealScreenWidth);
}

//assuming this is called while doUpdate is polled in a loop or at least after
//a doUpdate is called
Point ScanAggregator::getMarkerPt() const
{
    if(m_markers.size() > 0 && m_origins.size() >= 2)
    {
        Point p = m_markers.at(0);
        Point l = m_origins.at(1);
        //With wheels, detecting cross bar
        Point leftOffset = Point(INCHES_TO_METERS(6), INCHES_TO_METERS(-2.2), 0); 
        return (p-l)+leftOffset;
    }
    else
        return Point(-1, -1, -1);
}

//goes from positive x-axis centered view to origin in middle of screen
//with positive y up and positive x to right
Point ScanAggregator::transformCoordinates(const Point &p) const
{
    return Point(-p.y(), p.x(), 0) - 
	Point(METERS_TO_INCHES(m_idealScreenWidth/2.0f)+2, 0, 0.0f);
    return Point(-p.y(), p.x(), 0) - 
           Point(m_idealScreenWidth/2.0f, 
                 m_idealScreenWidth/2, 
                 0);
}

Point ScanAggregator::getMarkerPtInches() const
{
    Point p = getMarkerPt();
    return transformCoordinates(Point(
        METERS_TO_INCHES(p.x()),
        METERS_TO_INCHES(p.y()),
        METERS_TO_INCHES(p.z())));
}

Point ScanAggregator::getPointFromScanline(float depth, float angle)
{
    return Point(cos(angle)*depth, sin(angle)*depth, 0.0);
}

//extract foreground points, objects with maxMissing scanlines between
//them (or fewer) will be considered the same objects, also does a depth
//sanity check
std::vector<FGObject> ScanAggregator::extractForegroundObjects(
    int maxMissing, float maxObjDepthVariance)
{
    std::map<int,FGObject> labels;

    float lastValidDepth = -1;
    int numMissed = 0;
    int currentLabel = 0;
    bool inObject = false;
    float curAngle = m_minAngle;

    for(std::vector<float>::iterator i = m_avRanges.begin(); 
        i != m_avRanges.end(); ++i, curAngle += m_angleIncrement)
    {
        
        //we have a foreground point
        if((*i) != -1)
        {
            Point p = getPointFromScanline((*i), curAngle);

            //check to see if it's depth is within the tolerance compared
            //to the last valid foreground depth
            if(inObject && fabs((*i) - lastValidDepth) < maxObjDepthVariance)
            {
               labels[currentLabel].push_back(p); 
            }
            //different objects
            else if(inObject && 
                    fabs((*i) - lastValidDepth) >= maxObjDepthVariance)
               labels[++currentLabel].push_back(p); 
            else
               labels[currentLabel].push_back(p); 

            lastValidDepth = (*i);
            inObject = true;
        }
        //have a background point
        else
        {
            ++numMissed;
            //missed too many increment label, close last object
            if(numMissed > maxMissing)
            {
                ++currentLabel;
                inObject = false;
                numMissed = 0;
            }
        }
    }
    std::vector<FGObject> retVal;
    //at this point labels contains a map of object num -> vector of pts
    for(std::map<int,FGObject>::const_iterator i = labels.begin();
        i != labels.end(); ++i)
        retVal.push_back(i->second);

    return retVal;
}
