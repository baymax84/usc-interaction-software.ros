#ifndef BANDIT_H
#define BANDIT_H

#include <stdexcept>
#include <termios.h>
#include <string>
#include <stdint.h>

#include <boost/function.hpp>

#include <map>

#include "bandit/smartservo.h"

//! A namespace containing the bluesky bandit device driver
namespace bandit
{

// =============================================================================================================================================
template<class __Exception, class... __Args>
static void BANDIT_EXCEPT( std::string && msg, __Args&&... args );

// =============================================================================================================================================
//! Macro for defining an exception with a given parent ( std::runtime_error should be top parent )
#define DEF_EXCEPTION( name, parent ) \
    class name  : public parent { \
    public: \
    name( const char* msg ) : parent( msg ) {} \
    }

    //! A standard Bandit exception
    DEF_EXCEPTION( BanditException, smartservo::SmartServoException );
#undef DEF_EXCEPTION

// =============================================================================================================================================
// =============================================================================================================================================
static std::vector<std::pair<uint16_t, std::string> > const & getJointNamePairs()
{
    static const std::vector<std::pair<uint16_t, std::string> > joint_name_pairs
    {
        std::make_pair( (uint16_t) 0 , std::string( "head_tilt_joint"                        ) ),
        std::make_pair( (uint16_t) 1 , std::string( "head_pan_joint"                         ) ),
        std::make_pair( (uint16_t) 2 , std::string( "left_torso_shoulder_mounting_joint"     ) ),
        std::make_pair( (uint16_t) 3 , std::string( "left_shoulder_mounting_shoulder_joint"  ) ),
        std::make_pair( (uint16_t) 4 , std::string( "left_shoulder_bicep_joint"              ) ),
        std::make_pair( (uint16_t) 5 , std::string( "left_bicep_forearm_joint"               ) ),
        std::make_pair( (uint16_t) 6 , std::string( "left_forearm_wrist_joint"               ) ),
        std::make_pair( (uint16_t) 7 , std::string( "left_wrist_hand_joint"                  ) ),
        std::make_pair( (uint16_t) 8 , std::string( "left_hand_thumb_joint"                  ) ),
        std::make_pair( (uint16_t) 9 , std::string( "right_torso_shoulder_mounting_joint"    ) ),
        std::make_pair( (uint16_t) 10, std::string( "right_shoulder_mounting_shoulder_joint" ) ),
        std::make_pair( (uint16_t) 11, std::string( "right_shoulder_bicep_joint"             ) ),
        std::make_pair( (uint16_t) 12, std::string( "right_bicep_forearm_joint"              ) ),
        std::make_pair( (uint16_t) 13, std::string( "right_forearm_wrist_joint"              ) ),
        std::make_pair( (uint16_t) 14, std::string( "right_wrist_hand_joint"                 ) ),
        std::make_pair( (uint16_t) 15, std::string( "right_hand_thumb_joint"                 ) ),
        std::make_pair( (uint16_t) 16, std::string( "eyebrows_joint"                         ) ),
        std::make_pair( (uint16_t) 17, std::string( "mouth_top_joint"                        ) ),
        std::make_pair( (uint16_t) 18, std::string( "mouth_bottom_joint"                     ) )
    };

    return joint_name_pairs;
}

// =============================================================================================================================================
static std::map<std::string, uint16_t> const & getJointIdsMap()
{
    static bool initialized = false;
    static std::map<std::string, uint16_t> joint_ids_map;

    if( !initialized )
    {
        auto joint_name_pairs = getJointNamePairs();
        for( auto joint_name_it = joint_name_pairs.cbegin(); joint_name_it != joint_name_pairs.cend(); ++joint_name_it )
        {
            joint_ids_map[joint_name_it->second] = joint_name_it->first;
        }
        initialized = true;
    }

    return joint_ids_map;
}

// =============================================================================================================================================
static std::map<uint16_t, std::string> const & getJointNamesMap()
{
    static bool initialized = false;
    static std::map<uint16_t, std::string> joint_names_map;

    if( !initialized )
    {
        auto joint_name_pairs = getJointNamePairs();
        for( auto joint_name_it = joint_name_pairs.cbegin(); joint_name_it != joint_name_pairs.cend(); ++joint_name_it )
        {
            joint_names_map[joint_name_it->first] = joint_name_it->second;
        }
        initialized = true;
    }

    return joint_names_map;
}

// =============================================================================================================================================
class JointName
{
public:
    int16_t id_;
    std::string name_;

    JointName();
    JointName( int16_t const & id );
    JointName( std::string const & name );

    operator int16_t() const;
    operator std::string() const;
};

// =============================================================================================================================================
//! Generic bandit joint
/*!
 *  This joint may represent a bandit joint or servo
 */
class Joint
{
public:
    JointName               name;
    int16_t                 mod_id;
    smartservo::WhichJoint  which;
    smartservo::JointType   type;
    int8_t                  direction;
    double                  scale;
    double                  offset;
    double                  min;
    double                  max;

    Joint
    (
        JointName && name = JointName(),
        int16_t const & mod_id_ = -1,
        smartservo::WhichJoint const & which_ = smartservo::NO_JOINT,
        smartservo::JointType const & type_ = smartservo::NO_JOINT_TYPE,
        int8_t const & direction = 0,
        double const & scale_ = 0,
        double const & offset_ = 0,
        double const & min_ = 0,
        double const & max_ = 0
    );
};

// =============================================================================================================================================
//! A minimal driver class for communicating with bandit via serial
/*!
* This is relatively thin wrapper around the lower level Master to make
* joints more intuitive to work with.
*/
class Bandit
{
public:
    typedef boost::function<void()> _StateCallback;
    typedef std::map<uint16_t, Joint> _JointsMap;

private:
    //! Instance of Master module to be used to communicate with joints
    smartservo::MasterModule master_;

    //! use software joint limits?
    bool                    joint_limits_;

    _JointsMap joints_map_;

public:
    //! Constructor
    Bandit();

    //! Destructor;
    ~Bandit();

    template<class... __Args>
    void addJoint( __Args&&... args );

    //! value to set if we want to use software limits ( should be true unless calibrating joints )
    void useJointLimits( bool const & use_limits );

    // -----------------------------------------------------------------------------------------------------------------------------------------

    _JointsMap::iterator getJoint( JointName && joint_name );

    //! Return the type of a joint
    smartservo::JointType getJointType( JointName && joint_name );

    //! Return the name of a joint
    std::string getJointName( JointName && name );
    int16_t getJointId( JointName && name );

    // return the number of joints
    size_t getNumJoints();

    //! Return the minimum position of a joint
    double getJointMin( JointName && name );

    //! Return the maximum position of a joint
    double getJointMax( JointName && name );

    //! Get the value of a joint position
    double getJointPos( JointName && name );

    // -----------------------------------------------------------------------------------------------------------------------------------------

    //! Set the PID configuations for a particular joint.
    template<class... __Args>
    void setJointPIDConfig( JointName && name, __Args&&... args );

    //! Set the joint direction of a particular joint
    void setJointDirection( JointName && name, int8_t const & direction );

    //! Set the joint offset of a particular joint
    void setJointOffset( JointName && name, double const & offset );

    //! Set the joint position of a particular joint
    void setJointPos( JointName && name, double const & angle );

    // -----------------------------------------------------------------------------------------------------------------------------------------

    //! Open the port
    void openPort( std::string && port_name );

    //! Close the port
    void closePort();

    //! Callback which will be triggered whenever a state update is received
    void registerStateCB( _StateCallback && callback );

    //! Process a single pending incoming message
    /*!
     *  Processes a single pending incoming serial message from the
     *  master module.  This may invoke one of the registered callback
     *  functions as appropriate.
     *
     *  \param timeout   Timeout in microseconds ( 0 for indefinite )
     */
    void processIO( uint32_t const & timeout );

    void processPackets( bool const & wait = false );

    //! Send the joint positions to bandit
    void sendAllJointPos();

    //! Sync all the joint PID configurations to the master module
    void sendAllPIDConfigs();

    bool checkAllPIDConfigs( bool const & verbose = false );
};

#include <bandit/details/bandit_impl.h>

} // bandit

#endif
