/* Auto-generated by genmsg_cpp for file /home/wavy/workspace/ros/ardrone_tutorials/msg/StateData.msg */
#ifndef ARDRONE_TUTORIALS_MESSAGE_STATEDATA_H
#define ARDRONE_TUTORIALS_MESSAGE_STATEDATA_H
#include <string>
#include <vector>
#include <map>
#include <ostream>
#include "ros/serialization.h"
#include "ros/builtin_message_traits.h"
#include "ros/message_operations.h"
#include "ros/time.h"

#include "ros/macros.h"

#include "ros/assert.h"


namespace ardrone_tutorials
{
template <class ContainerAllocator>
struct StateData_ {
  typedef StateData_<ContainerAllocator> Type;

  StateData_()
  : x(0.0)
  , y(0.0)
  , z(0.0)
  , roll(0.0)
  , pitch(0.0)
  , yaw(0.0)
  , vx(0.0)
  , vy(0.0)
  , vz(0.0)
  , ax(0.0)
  , ay(0.0)
  , az(0.0)
  {
  }

  StateData_(const ContainerAllocator& _alloc)
  : x(0.0)
  , y(0.0)
  , z(0.0)
  , roll(0.0)
  , pitch(0.0)
  , yaw(0.0)
  , vx(0.0)
  , vy(0.0)
  , vz(0.0)
  , ax(0.0)
  , ay(0.0)
  , az(0.0)
  {
  }

  typedef double _x_type;
  double x;

  typedef double _y_type;
  double y;

  typedef double _z_type;
  double z;

  typedef double _roll_type;
  double roll;

  typedef double _pitch_type;
  double pitch;

  typedef double _yaw_type;
  double yaw;

  typedef double _vx_type;
  double vx;

  typedef double _vy_type;
  double vy;

  typedef double _vz_type;
  double vz;

  typedef double _ax_type;
  double ax;

  typedef double _ay_type;
  double ay;

  typedef double _az_type;
  double az;


  typedef boost::shared_ptr< ::ardrone_tutorials::StateData_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::ardrone_tutorials::StateData_<ContainerAllocator>  const> ConstPtr;
  boost::shared_ptr<std::map<std::string, std::string> > __connection_header;
}; // struct StateData
typedef  ::ardrone_tutorials::StateData_<std::allocator<void> > StateData;

typedef boost::shared_ptr< ::ardrone_tutorials::StateData> StateDataPtr;
typedef boost::shared_ptr< ::ardrone_tutorials::StateData const> StateDataConstPtr;


template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const  ::ardrone_tutorials::StateData_<ContainerAllocator> & v)
{
  ros::message_operations::Printer< ::ardrone_tutorials::StateData_<ContainerAllocator> >::stream(s, "", v);
  return s;}

} // namespace ardrone_tutorials

namespace ros
{
namespace message_traits
{
template<class ContainerAllocator> struct IsMessage< ::ardrone_tutorials::StateData_<ContainerAllocator> > : public TrueType {};
template<class ContainerAllocator> struct IsMessage< ::ardrone_tutorials::StateData_<ContainerAllocator>  const> : public TrueType {};
template<class ContainerAllocator>
struct MD5Sum< ::ardrone_tutorials::StateData_<ContainerAllocator> > {
  static const char* value() 
  {
    return "1d95cd8bc3cb2123b24beec59ccd8555";
  }

  static const char* value(const  ::ardrone_tutorials::StateData_<ContainerAllocator> &) { return value(); } 
  static const uint64_t static_value1 = 0x1d95cd8bc3cb2123ULL;
  static const uint64_t static_value2 = 0xb24beec59ccd8555ULL;
};

template<class ContainerAllocator>
struct DataType< ::ardrone_tutorials::StateData_<ContainerAllocator> > {
  static const char* value() 
  {
    return "ardrone_tutorials/StateData";
  }

  static const char* value(const  ::ardrone_tutorials::StateData_<ContainerAllocator> &) { return value(); } 
};

template<class ContainerAllocator>
struct Definition< ::ardrone_tutorials::StateData_<ContainerAllocator> > {
  static const char* value() 
  {
    return "# StateData\n\
# Data necessary to communicate the current/desired state of the drone\n\
\n\
# Linear Position (m)\n\
float64 x\n\
float64 y\n\
float64 z\n\
\n\
# Angular Position (rads)\n\
float64 roll\n\
float64 pitch\n\
float64 yaw\n\
\n\
# Linear Velocities (m/s)\n\
float64 vx\n\
float64 vy\n\
float64 vz\n\
\n\
# Vertical Accelerations (m/s/s)\n\
float64 ax\n\
float64 ay\n\
float64 az\n\
\n\
";
  }

  static const char* value(const  ::ardrone_tutorials::StateData_<ContainerAllocator> &) { return value(); } 
};

template<class ContainerAllocator> struct IsFixedSize< ::ardrone_tutorials::StateData_<ContainerAllocator> > : public TrueType {};
} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

template<class ContainerAllocator> struct Serializer< ::ardrone_tutorials::StateData_<ContainerAllocator> >
{
  template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
  {
    stream.next(m.x);
    stream.next(m.y);
    stream.next(m.z);
    stream.next(m.roll);
    stream.next(m.pitch);
    stream.next(m.yaw);
    stream.next(m.vx);
    stream.next(m.vy);
    stream.next(m.vz);
    stream.next(m.ax);
    stream.next(m.ay);
    stream.next(m.az);
  }

  ROS_DECLARE_ALLINONE_SERIALIZER;
}; // struct StateData_
} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::ardrone_tutorials::StateData_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const  ::ardrone_tutorials::StateData_<ContainerAllocator> & v) 
  {
    s << indent << "x: ";
    Printer<double>::stream(s, indent + "  ", v.x);
    s << indent << "y: ";
    Printer<double>::stream(s, indent + "  ", v.y);
    s << indent << "z: ";
    Printer<double>::stream(s, indent + "  ", v.z);
    s << indent << "roll: ";
    Printer<double>::stream(s, indent + "  ", v.roll);
    s << indent << "pitch: ";
    Printer<double>::stream(s, indent + "  ", v.pitch);
    s << indent << "yaw: ";
    Printer<double>::stream(s, indent + "  ", v.yaw);
    s << indent << "vx: ";
    Printer<double>::stream(s, indent + "  ", v.vx);
    s << indent << "vy: ";
    Printer<double>::stream(s, indent + "  ", v.vy);
    s << indent << "vz: ";
    Printer<double>::stream(s, indent + "  ", v.vz);
    s << indent << "ax: ";
    Printer<double>::stream(s, indent + "  ", v.ax);
    s << indent << "ay: ";
    Printer<double>::stream(s, indent + "  ", v.ay);
    s << indent << "az: ";
    Printer<double>::stream(s, indent + "  ", v.az);
  }
};


} // namespace message_operations
} // namespace ros

#endif // ARDRONE_TUTORIALS_MESSAGE_STATEDATA_H

