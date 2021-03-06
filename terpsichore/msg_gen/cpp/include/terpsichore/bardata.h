/* Auto-generated by genmsg_cpp for file /home/wavy/workspace/ros/terpsichore/msg/bardata.msg */
#ifndef TERPSICHORE_MESSAGE_BARDATA_H
#define TERPSICHORE_MESSAGE_BARDATA_H
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

#include "terpsichore/pair.h"
#include "terpsichore/pair.h"

namespace terpsichore
{
template <class ContainerAllocator>
struct bardata_ {
  typedef bardata_<ContainerAllocator> Type;

  bardata_()
  : beats()
  , events()
  {
  }

  bardata_(const ContainerAllocator& _alloc)
  : beats(_alloc)
  , events(_alloc)
  {
  }

  typedef std::vector< ::terpsichore::pair_<ContainerAllocator> , typename ContainerAllocator::template rebind< ::terpsichore::pair_<ContainerAllocator> >::other >  _beats_type;
  std::vector< ::terpsichore::pair_<ContainerAllocator> , typename ContainerAllocator::template rebind< ::terpsichore::pair_<ContainerAllocator> >::other >  beats;

  typedef std::vector< ::terpsichore::pair_<ContainerAllocator> , typename ContainerAllocator::template rebind< ::terpsichore::pair_<ContainerAllocator> >::other >  _events_type;
  std::vector< ::terpsichore::pair_<ContainerAllocator> , typename ContainerAllocator::template rebind< ::terpsichore::pair_<ContainerAllocator> >::other >  events;


  typedef boost::shared_ptr< ::terpsichore::bardata_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::terpsichore::bardata_<ContainerAllocator>  const> ConstPtr;
  boost::shared_ptr<std::map<std::string, std::string> > __connection_header;
}; // struct bardata
typedef  ::terpsichore::bardata_<std::allocator<void> > bardata;

typedef boost::shared_ptr< ::terpsichore::bardata> bardataPtr;
typedef boost::shared_ptr< ::terpsichore::bardata const> bardataConstPtr;


template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const  ::terpsichore::bardata_<ContainerAllocator> & v)
{
  ros::message_operations::Printer< ::terpsichore::bardata_<ContainerAllocator> >::stream(s, "", v);
  return s;}

} // namespace terpsichore

namespace ros
{
namespace message_traits
{
template<class ContainerAllocator> struct IsMessage< ::terpsichore::bardata_<ContainerAllocator> > : public TrueType {};
template<class ContainerAllocator> struct IsMessage< ::terpsichore::bardata_<ContainerAllocator>  const> : public TrueType {};
template<class ContainerAllocator>
struct MD5Sum< ::terpsichore::bardata_<ContainerAllocator> > {
  static const char* value() 
  {
    return "9c113914757a1b063ff8abb1eea20f57";
  }

  static const char* value(const  ::terpsichore::bardata_<ContainerAllocator> &) { return value(); } 
  static const uint64_t static_value1 = 0x9c113914757a1b06ULL;
  static const uint64_t static_value2 = 0x3ff8abb1eea20f57ULL;
};

template<class ContainerAllocator>
struct DataType< ::terpsichore::bardata_<ContainerAllocator> > {
  static const char* value() 
  {
    return "terpsichore/bardata";
  }

  static const char* value(const  ::terpsichore::bardata_<ContainerAllocator> &) { return value(); } 
};

template<class ContainerAllocator>
struct Definition< ::terpsichore::bardata_<ContainerAllocator> > {
  static const char* value() 
  {
    return "pair[] beats\n\
pair[] events\n\
================================================================================\n\
MSG: terpsichore/pair\n\
float64 t\n\
float64[] data\n\
";
  }

  static const char* value(const  ::terpsichore::bardata_<ContainerAllocator> &) { return value(); } 
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

template<class ContainerAllocator> struct Serializer< ::terpsichore::bardata_<ContainerAllocator> >
{
  template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
  {
    stream.next(m.beats);
    stream.next(m.events);
  }

  ROS_DECLARE_ALLINONE_SERIALIZER;
}; // struct bardata_
} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::terpsichore::bardata_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const  ::terpsichore::bardata_<ContainerAllocator> & v) 
  {
    s << indent << "beats[]" << std::endl;
    for (size_t i = 0; i < v.beats.size(); ++i)
    {
      s << indent << "  beats[" << i << "]: ";
      s << std::endl;
      s << indent;
      Printer< ::terpsichore::pair_<ContainerAllocator> >::stream(s, indent + "    ", v.beats[i]);
    }
    s << indent << "events[]" << std::endl;
    for (size_t i = 0; i < v.events.size(); ++i)
    {
      s << indent << "  events[" << i << "]: ";
      s << std::endl;
      s << indent;
      Printer< ::terpsichore::pair_<ContainerAllocator> >::stream(s, indent + "    ", v.events[i]);
    }
  }
};


} // namespace message_operations
} // namespace ros

#endif // TERPSICHORE_MESSAGE_BARDATA_H

