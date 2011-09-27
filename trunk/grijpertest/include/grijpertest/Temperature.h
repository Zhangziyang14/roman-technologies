/* Auto-generated by genmsg_cpp for file /home/hans/Minor-Robotica/ros_workspace/grijpertest/msg/Temperature.msg */
#ifndef GRIJPERTEST_MESSAGE_TEMPERATURE_H
#define GRIJPERTEST_MESSAGE_TEMPERATURE_H
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


namespace grijpertest
{
template <class ContainerAllocator>
struct Temperature_ {
  typedef Temperature_<ContainerAllocator> Type;

  Temperature_()
  : temperature(0)
  {
  }

  Temperature_(const ContainerAllocator& _alloc)
  : temperature(0)
  {
  }

  typedef int32_t _temperature_type;
  int32_t temperature;


private:
  static const char* __s_getDataType_() { return "grijpertest/Temperature"; }
public:
  ROS_DEPRECATED static const std::string __s_getDataType() { return __s_getDataType_(); }

  ROS_DEPRECATED const std::string __getDataType() const { return __s_getDataType_(); }

private:
  static const char* __s_getMD5Sum_() { return "b3342cc2976d1fdf1293007ddd8de1eb"; }
public:
  ROS_DEPRECATED static const std::string __s_getMD5Sum() { return __s_getMD5Sum_(); }

  ROS_DEPRECATED const std::string __getMD5Sum() const { return __s_getMD5Sum_(); }

private:
  static const char* __s_getMessageDefinition_() { return "int32 temperature\n\
\n\
"; }
public:
  ROS_DEPRECATED static const std::string __s_getMessageDefinition() { return __s_getMessageDefinition_(); }

  ROS_DEPRECATED const std::string __getMessageDefinition() const { return __s_getMessageDefinition_(); }

  ROS_DEPRECATED virtual uint8_t *serialize(uint8_t *write_ptr, uint32_t seq) const
  {
    ros::serialization::OStream stream(write_ptr, 1000000000);
    ros::serialization::serialize(stream, temperature);
    return stream.getData();
  }

  ROS_DEPRECATED virtual uint8_t *deserialize(uint8_t *read_ptr)
  {
    ros::serialization::IStream stream(read_ptr, 1000000000);
    ros::serialization::deserialize(stream, temperature);
    return stream.getData();
  }

  ROS_DEPRECATED virtual uint32_t serializationLength() const
  {
    uint32_t size = 0;
    size += ros::serialization::serializationLength(temperature);
    return size;
  }

  typedef boost::shared_ptr< ::grijpertest::Temperature_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::grijpertest::Temperature_<ContainerAllocator>  const> ConstPtr;
  boost::shared_ptr<std::map<std::string, std::string> > __connection_header;
}; // struct Temperature
typedef  ::grijpertest::Temperature_<std::allocator<void> > Temperature;

typedef boost::shared_ptr< ::grijpertest::Temperature> TemperaturePtr;
typedef boost::shared_ptr< ::grijpertest::Temperature const> TemperatureConstPtr;


template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const  ::grijpertest::Temperature_<ContainerAllocator> & v)
{
  ros::message_operations::Printer< ::grijpertest::Temperature_<ContainerAllocator> >::stream(s, "", v);
  return s;}

} // namespace grijpertest

namespace ros
{
namespace message_traits
{
template<class ContainerAllocator> struct IsMessage< ::grijpertest::Temperature_<ContainerAllocator> > : public TrueType {};
template<class ContainerAllocator> struct IsMessage< ::grijpertest::Temperature_<ContainerAllocator>  const> : public TrueType {};
template<class ContainerAllocator>
struct MD5Sum< ::grijpertest::Temperature_<ContainerAllocator> > {
  static const char* value() 
  {
    return "b3342cc2976d1fdf1293007ddd8de1eb";
  }

  static const char* value(const  ::grijpertest::Temperature_<ContainerAllocator> &) { return value(); } 
  static const uint64_t static_value1 = 0xb3342cc2976d1fdfULL;
  static const uint64_t static_value2 = 0x1293007ddd8de1ebULL;
};

template<class ContainerAllocator>
struct DataType< ::grijpertest::Temperature_<ContainerAllocator> > {
  static const char* value() 
  {
    return "grijpertest/Temperature";
  }

  static const char* value(const  ::grijpertest::Temperature_<ContainerAllocator> &) { return value(); } 
};

template<class ContainerAllocator>
struct Definition< ::grijpertest::Temperature_<ContainerAllocator> > {
  static const char* value() 
  {
    return "int32 temperature\n\
\n\
";
  }

  static const char* value(const  ::grijpertest::Temperature_<ContainerAllocator> &) { return value(); } 
};

template<class ContainerAllocator> struct IsFixedSize< ::grijpertest::Temperature_<ContainerAllocator> > : public TrueType {};
} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

template<class ContainerAllocator> struct Serializer< ::grijpertest::Temperature_<ContainerAllocator> >
{
  template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
  {
    stream.next(m.temperature);
  }

  ROS_DECLARE_ALLINONE_SERIALIZER;
}; // struct Temperature_
} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::grijpertest::Temperature_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const  ::grijpertest::Temperature_<ContainerAllocator> & v) 
  {
    s << indent << "temperature: ";
    Printer<int32_t>::stream(s, indent + "  ", v.temperature);
  }
};


} // namespace message_operations
} // namespace ros

#endif // GRIJPERTEST_MESSAGE_TEMPERATURE_H

