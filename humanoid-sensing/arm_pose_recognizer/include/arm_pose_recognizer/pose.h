#ifndef ARMPOSERECOGNIZER_POSE_H_
#define ARMPOSERECOGNIZER_POSE_H_

#include <quickdev/vector.h>

template<class __Data>
class Pose : public quickdev::VectorWrapper<8, __Data, Pose<__Data> >
{
public:
    typedef __Data _Data;

    __Data & l_elbow;
    __Data & r_elbow;
    __Data & l_hand;
    __Data & r_hand;

    __Data & l_elbow_mirror;
    __Data & r_elbow_mirror;
    __Data & l_hand_mirror;
    __Data & r_hand_mirror;

    typedef quickdev::VectorWrapper<8, __Data, Pose<__Data> > _Storage;

    Pose( Pose const & other )
    :
        _Storage(),
        l_elbow( other.l_elbow ),
        r_elbow( other.r_elbow ),
        l_hand( other.l_hand ),
        r_hand( other.r_hand ),

        l_elbow_mirror( other.l_elbow_mirror ),
        r_elbow_mirror( other.r_elbow_mirror ),
        l_hand_mirror( other.l_hand_mirror ),
        r_hand_mirror( other.r_hand_mirror )
    {
        this->copyFrom( other );
    }

    template<class... __Args>
    Pose( __Args&&... args )
    :
        _Storage( std::forward<__Args>( args )... ),
        l_elbow( this->at( 0 ) ),
        r_elbow( this->at( 1 ) ),
        l_hand( this->at( 2 ) ),
        r_hand( this->at( 3 ) ),
        l_elbow_mirror( this->at( 4 ) ),
        r_elbow_mirror( this->at( 5 ) ),
        l_hand_mirror( this->at( 6 ) ),
        r_hand_mirror( this->at( 7 ) )
    {
        //
    }

    Pose & operator=( Pose const & other )
    {
        this->copyFrom( other );

        l_elbow = other.l_elbow;
        r_elbow = other.r_elbow;
        l_hand = other.l_hand;
        r_hand = other.r_hand;

        l_elbow_mirror = other.l_elbow_mirror;
        r_elbow_mirror = other.r_elbow_mirror;
        l_hand_mirror = other.l_hand_mirror;
        r_hand_mirror = other.r_hand_mirror;

        return *this;
    }

    double calculateForearmLength() const
    {
        return ( l_elbow.distance(  l_hand ) + r_elbow.distance( r_hand ) ) / 2;
    }

    Pose & mirror()
    {
        l_elbow_mirror = r_elbow;
        r_elbow_mirror = l_elbow;
        l_hand_mirror = r_hand;
        r_hand_mirror = l_hand;

        for( size_t i = 4; i < this->size(); ++i )
        {
            auto & elem = *(this->begin() + i);

            elem.setY( elem.getY() * -1 );
        }

        return *this;
    }

    Pose & normalize()
    {
        auto const forearm_length = calculateForearmLength();

        for( size_t i = 0; i < 4; ++i )
        {
            auto elem = this->begin() + i;

            *elem /= forearm_length;
        }

        mirror();

        return *this;
    }

    /*_MarkerArrayMsg toMarkerMsg( double const & scale )
    {
        _MarkerArrayMsg markers;

        _MarkerMsg point_marker;

        //point_marker.header.frame_id = "/world";
        //point_marker.header.stamp = ros::Time::now();
        point_marker.action = _MarkerMsg::ADD;
        point_marker.lifetime = ros::Duration( 0.1 );
        point_marker.pose.orientation.w = 1.0;
        point_marker.color = current_color;

        point_marker.ns = "static_gesture";

        point_marker.type = _MarkerMsg::SPHERE_LIST;
        point_marker.scale.x = point_marker.scale.y
                             = point_marker.scale.z
                             = scale;

        point_marker.pose.position.x = mean.x;
        point_marker.pose.position.y = mean.y;
        point_marker.pose.position.z = mean.z;

        markers.markers.push_back( point_marker );

        return markers;
    }*/
};

#endif // ARMPOSERECOGNIZER_POSE_H_
