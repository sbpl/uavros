/* Auto-generated by genmsg_cpp for file /home/venkat/ros_workspace/uavros/uav_msgs/msg/mode_msg.msg */
#ifndef UAV_MSGS_MESSAGE_MODE_MSG_H
#define UAV_MSGS_MESSAGE_MODE_MSG_H
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


namespace uav_msgs
{
template <class ContainerAllocator>
struct mode_msg_ {
  typedef mode_msg_<ContainerAllocator> Type;

  mode_msg_()
  : marker(0)
  , mode(0)
  {
  }

  mode_msg_(const ContainerAllocator& _alloc)
  : marker(0)
  , mode(0)
  {
  }

  typedef int32_t _marker_type;
  int32_t marker;

  typedef int32_t _mode_type;
  int32_t mode;


private:
  static const char* __s_getDataType_() { return "uav_msgs/mode_msg"; }
public:
  ROS_DEPRECATED static const std::string __s_getDataType() { return __s_getDataType_(); }

  ROS_DEPRECATED const std::string __getDataType() const { return __s_getDataType_(); }

private:
  static const char* __s_getMD5Sum_() { return "d4076e03c4c3e3bb1339b01b12ca8e35"; }
public:
  ROS_DEPRECATED static const std::string __s_getMD5Sum() { return __s_getMD5Sum_(); }

  ROS_DEPRECATED const std::string __getMD5Sum() const { return __s_getMD5Sum_(); }

private:
  static const char* __s_getMessageDefinition_() { return "int32 marker\n\
int32 mode\n\
\n\
"; }
public:
  ROS_DEPRECATED static const std::string __s_getMessageDefinition() { return __s_getMessageDefinition_(); }

  ROS_DEPRECATED const std::string __getMessageDefinition() const { return __s_getMessageDefinition_(); }

  ROS_DEPRECATED virtual uint8_t *serialize(uint8_t *write_ptr, uint32_t seq) const
  {
    ros::serialization::OStream stream(write_ptr, 1000000000);
    ros::serialization::serialize(stream, marker);
    ros::serialization::serialize(stream, mode);
    return stream.getData();
  }

  ROS_DEPRECATED virtual uint8_t *deserialize(uint8_t *read_ptr)
  {
    ros::serialization::IStream stream(read_ptr, 1000000000);
    ros::serialization::deserialize(stream, marker);
    ros::serialization::deserialize(stream, mode);
    return stream.getData();
  }

  ROS_DEPRECATED virtual uint32_t serializationLength() const
  {
    uint32_t size = 0;
    size += ros::serialization::serializationLength(marker);
    size += ros::serialization::serializationLength(mode);
    return size;
  }

  typedef boost::shared_ptr< ::uav_msgs::mode_msg_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::uav_msgs::mode_msg_<ContainerAllocator>  const> ConstPtr;
  boost::shared_ptr<std::map<std::string, std::string> > __connection_header;
}; // struct mode_msg
typedef  ::uav_msgs::mode_msg_<std::allocator<void> > mode_msg;

typedef boost::shared_ptr< ::uav_msgs::mode_msg> mode_msgPtr;
typedef boost::shared_ptr< ::uav_msgs::mode_msg const> mode_msgConstPtr;


template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const  ::uav_msgs::mode_msg_<ContainerAllocator> & v)
{
  ros::message_operations::Printer< ::uav_msgs::mode_msg_<ContainerAllocator> >::stream(s, "", v);
  return s;}

} // namespace uav_msgs

namespace ros
{
namespace message_traits
{
template<class ContainerAllocator> struct IsMessage< ::uav_msgs::mode_msg_<ContainerAllocator> > : public TrueType {};
template<class ContainerAllocator> struct IsMessage< ::uav_msgs::mode_msg_<ContainerAllocator>  const> : public TrueType {};
template<class ContainerAllocator>
struct MD5Sum< ::uav_msgs::mode_msg_<ContainerAllocator> > {
  static const char* value() 
  {
    return "d4076e03c4c3e3bb1339b01b12ca8e35";
  }

  static const char* value(const  ::uav_msgs::mode_msg_<ContainerAllocator> &) { return value(); } 
  static const uint64_t static_value1 = 0xd4076e03c4c3e3bbULL;
  static const uint64_t static_value2 = 0x1339b01b12ca8e35ULL;
};

template<class ContainerAllocator>
struct DataType< ::uav_msgs::mode_msg_<ContainerAllocator> > {
  static const char* value() 
  {
    return "uav_msgs/mode_msg";
  }

  static const char* value(const  ::uav_msgs::mode_msg_<ContainerAllocator> &) { return value(); } 
};

template<class ContainerAllocator>
struct Definition< ::uav_msgs::mode_msg_<ContainerAllocator> > {
  static const char* value() 
  {
    return "int32 marker\n\
int32 mode\n\
\n\
";
  }

  static const char* value(const  ::uav_msgs::mode_msg_<ContainerAllocator> &) { return value(); } 
};

template<class ContainerAllocator> struct IsFixedSize< ::uav_msgs::mode_msg_<ContainerAllocator> > : public TrueType {};
} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

template<class ContainerAllocator> struct Serializer< ::uav_msgs::mode_msg_<ContainerAllocator> >
{
  template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
  {
    stream.next(m.marker);
    stream.next(m.mode);
  }

  ROS_DECLARE_ALLINONE_SERIALIZER;
}; // struct mode_msg_
} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::uav_msgs::mode_msg_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const  ::uav_msgs::mode_msg_<ContainerAllocator> & v) 
  {
    s << indent << "marker: ";
    Printer<int32_t>::stream(s, indent + "  ", v.marker);
    s << indent << "mode: ";
    Printer<int32_t>::stream(s, indent + "  ", v.mode);
  }
};


} // namespace message_operations
} // namespace ros

#endif // UAV_MSGS_MESSAGE_MODE_MSG_H

