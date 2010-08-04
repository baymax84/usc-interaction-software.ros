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

  //! Macro for defining an exception with a given parent (std::runtime_error should be top parent)
#define DEF_EXCEPTION(name, parent) \
  class name  : public parent { \
  public: \
    name(const char* msg) : parent(msg) {} \
  }
  
  //! A standard Bandit exception
  DEF_EXCEPTION(BanditException, smartservo::SmartServoException);

#undef DEF_EXCEPTION

  //! A minimal driver class for communicating with bandit via serial
  /*! 
   * This is relatively thin wrapper around the lower level Master to make
   * joints more intuitive to work with.
   */
  class Bandit
  {
  private:
    //! Instance of Master module to be used to communicate with joints
    smartservo::MasterModule master_;

    //! use software joint limits?
    bool                    joint_limits_;

    //! Generic bandit joint
    /*! 
     *  This joint may represent a bandit joint or servo
     */
    class Joint
    {
    public:
      std::string             name;
      std::string             ros_name;
      int16_t                 mod_id;
      smartservo::WhichJoint  which;
      smartservo::JointType   type;
      int8_t                  direction;
      double                  scale;
      double                  offset;
      double                  min;
      double                  max;

      Joint() : name(""), mod_id(-1), which(smartservo::NO_JOINT), type(smartservo::NO_JOINT_TYPE),
                direction(0), scale(0.0), offset(0), min(0), max(0) { }
    };

    std::map<uint16_t, Joint> joints_;

  public:
    //! Constructor
    Bandit();
    
    //! Destructor;
    ~Bandit();

    void addJoint(int id, std::string name, std::string rname, int mod_id, smartservo::WhichJoint which, smartservo::JointType type,
                          int8_t direction, double scale, double offset, double min, double max);

  
    //! value to set if we want to use software limits (should be true unless calibrating joints)
    void useJointLimits( bool use_limits );
  
    //! Return the type of a joint
    smartservo::JointType getJointType(int id);

    //! Return the name of a joint
    std::string getJointName(int id);
    std::string getJointRosName(int id);

    //! Return the minimum position of a joint
    double getJointMin(int id);

    //! Return the maximum position of a joint
    double getJointMax(int id);

    //! Open the port
    void openPort(const char* port_name) { master_.openPort(port_name); }

    //! Close the port
    void closePort() { master_.closePort(); }
    
    //! Callback which will be triggered whenever a state update is received
    void registerStateCB(boost::function<void()>);
    
    //! Process a single pending incoming message
    /*! 
     *  Processes a single pending incoming serial message from the
     *  master module.  This may invoke one of the registered callback
     *  functions as appropriate.
     *
     *  \param timeout   Timeout in microseconds (0 for indefinite)
     */
    void processIO(uint32_t timeout) { master_.processIO(timeout); }

    void processPackets(bool wait = false) { master_.processPackets(wait); }


    //! Set the PID configuations for a particular joint.
    void setJointPIDConfig(int16_t id,
                           int16_t p, int16_t i, int16_t d,
                           int16_t i_min, int16_t i_max,
                           uint8_t e_min, int16_t offset);

    //! Sync all the joint PID configurations to the master module
    void sendAllPIDConfigs() { master_.sendAllPIDConfigs(); }

    bool checkAllPIDConfigs(bool verbose=false) { return master_.checkAllPIDConfigs(verbose); }

    //! Set the joint direction of a particular joint
    void setJointDirection(uint16_t id, int8_t direction);

    //! Set the joint offset of a particular joint
    void setJointOffset(uint16_t id, double offset);

    //! Set the joint position of a particular joint
    void setJointPos(uint16_t id, double angle);

    //! Get the value of a joint position
    double getJointPos(uint16_t id);

    //! Send the joint positions to bandit
    void sendAllJointPos() {   master_.sendAllJointPos(); master_.sendAllJointServoPos(); }
  };
};

#endif
