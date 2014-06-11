/* Auto-generated by genmsg_cpp for file /home/wavy/workspace/ros/terpsichore/msg/pair.msg */
#ifndef TERPSICHORE_MESSAGE_PAIR_H
#define TERPSICHORE_MESSAGE_PAIR_H
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


namespace terpsichore
{
template <class ContainerAllocator>
struct pair_ {
  typedef pair_<ContainerAllocator> Type;

  pair_()
  : t(0.0)
  , data()
  {
  }

  pair_(const ContainerAllocator& _alloc)
  : t(0.0)
  , data(_alloc)
  {
  }

  typedef double _t_type;
  double t;

  typedef std::vector<double, typename ContainerAllocator::template rebind<double>::other >  _data_type;
  std::vector<double, typename ContainerAllocator::template rebind<double>::other >  data;


  typedef boost::shared_ptr< ::terpsichore::pair_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::terpsichore::pair_<ContainerAllocator>  const> ConstPtr;
  boost::shared_ptr<std::map<std::string, std::string> > __connection_header;
}; // struct pair
typedef  ::terpsichore::pair_<std::allocator<void> > pair;

typedef boost::shared_ptr< ::terpsichore::pair> pairPtr;
typedef boost::shared_ptr< ::terpsichore::pair const> pairConstPtr;


template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const  ::terpsichore::pair_<ContainerAllocator> & v)
{
  ros::message_operations::Printer< ::terpsichore::pair_<ContainerAllocator> >::stream(s, "", v);
  return s;}

} // namespace terpsichore

namespace ros
{
namespace message_traits
{
template<class ContainerAllocator> struct IsMessage< ::terpsichore::pair_<ContainerAllocator> > : public TrueType {};
template<class ContainerAllocator> struct IsMessage< ::terpsichore::pair_<ContainerAllocator>  const> : public TrueType {};
template<class ContainerAllocator>
struct MD5Sum< ::terpsichore::pair_<ContainerAllocator> > {
  static const char* value() 
  {
    return "4704d1e9bdec53dad180dbd02389b120";
  }

  static const char* value(const  ::terpsichore::pair_<ContainerAllocator> &) { return value(); } 
  static const uint64_t static_value1 = 0x4704d1e9bdec53daULL;
  static const uint64_t static_value2 = 0xd180dbd02389b120ULL;
};

template<class ContainerAllocator>
struct DataType< ::terpsichore::pair_<ContainerAllocator> > {
  static const char* value() 
  {
    return "terpsichore/pair";
  }

  static const char* value(const  ::terpsichore::pair_<ContainerAllocator> &) { return value(); } 
};

template<class ContainerAllocator>
struct Definition< ::terpsichore::pair_<ContainerAllocator> > {
  static const char* value() 
  {
    return "float64 t\n\
float64[] data\n\
";
  }

  static const char* value(const  ::terpsichore::pair_<ContainerAllocator> &) { return value(); } 
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

template<class ContainerAllocator> struct Serializer< ::terpsichore::pair_<ContainerAllocator> >
{
  template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
  {
    stream.next(m.t);
    stream.next(m.data);
  }

  ROS_DECLARE_ALLINONE_SERIALIZER;
}; // struct pair_
} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::terpsichore::pair_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const  ::terpsichore::pair_<ContainerAllocator> & v) 
  {
    s << indent << "t: ";
    Printer<double>::stream(s, indent + "  ", v.t);
    s << indent << "data[]" << std::endl;
    for (size_t i = 0; i < v.data.size(); ++i)
    {
      s << indent << "  data[" << i << "]: ";
      Printer<double>::stream(s, indent + "  ", v.data[i]);
    }
  }
};


} // namespace message_operations
} // namespace ros

#endif // TERPSICHORE_MESSAGE_PAIR_H

