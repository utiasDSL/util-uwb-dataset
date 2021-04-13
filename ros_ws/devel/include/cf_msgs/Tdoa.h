// Generated by gencpp from file cf_msgs/Tdoa.msg
// DO NOT EDIT!


#ifndef CF_MSGS_MESSAGE_TDOA_H
#define CF_MSGS_MESSAGE_TDOA_H


#include <string>
#include <vector>
#include <map>

#include <ros/types.h>
#include <ros/serialization.h>
#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>

#include <std_msgs/Header.h>

namespace cf_msgs
{
template <class ContainerAllocator>
struct Tdoa_
{
  typedef Tdoa_<ContainerAllocator> Type;

  Tdoa_()
    : header()
    , idA(0)
    , idB(0)
    , data(0.0)  {
    }
  Tdoa_(const ContainerAllocator& _alloc)
    : header(_alloc)
    , idA(0)
    , idB(0)
    , data(0.0)  {
  (void)_alloc;
    }



   typedef  ::std_msgs::Header_<ContainerAllocator>  _header_type;
  _header_type header;

   typedef int32_t _idA_type;
  _idA_type idA;

   typedef int32_t _idB_type;
  _idB_type idB;

   typedef float _data_type;
  _data_type data;





  typedef boost::shared_ptr< ::cf_msgs::Tdoa_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::cf_msgs::Tdoa_<ContainerAllocator> const> ConstPtr;

}; // struct Tdoa_

typedef ::cf_msgs::Tdoa_<std::allocator<void> > Tdoa;

typedef boost::shared_ptr< ::cf_msgs::Tdoa > TdoaPtr;
typedef boost::shared_ptr< ::cf_msgs::Tdoa const> TdoaConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::cf_msgs::Tdoa_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::cf_msgs::Tdoa_<ContainerAllocator> >::stream(s, "", v);
return s;
}


template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator==(const ::cf_msgs::Tdoa_<ContainerAllocator1> & lhs, const ::cf_msgs::Tdoa_<ContainerAllocator2> & rhs)
{
  return lhs.header == rhs.header &&
    lhs.idA == rhs.idA &&
    lhs.idB == rhs.idB &&
    lhs.data == rhs.data;
}

template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator!=(const ::cf_msgs::Tdoa_<ContainerAllocator1> & lhs, const ::cf_msgs::Tdoa_<ContainerAllocator2> & rhs)
{
  return !(lhs == rhs);
}


} // namespace cf_msgs

namespace ros
{
namespace message_traits
{





template <class ContainerAllocator>
struct IsFixedSize< ::cf_msgs::Tdoa_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::cf_msgs::Tdoa_<ContainerAllocator> const>
  : FalseType
  { };

template <class ContainerAllocator>
struct IsMessage< ::cf_msgs::Tdoa_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::cf_msgs::Tdoa_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::cf_msgs::Tdoa_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::cf_msgs::Tdoa_<ContainerAllocator> const>
  : TrueType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::cf_msgs::Tdoa_<ContainerAllocator> >
{
  static const char* value()
  {
    return "b3caf99b788d7132e40204b4bdeef95d";
  }

  static const char* value(const ::cf_msgs::Tdoa_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0xb3caf99b788d7132ULL;
  static const uint64_t static_value2 = 0xe40204b4bdeef95dULL;
};

template<class ContainerAllocator>
struct DataType< ::cf_msgs::Tdoa_<ContainerAllocator> >
{
  static const char* value()
  {
    return "cf_msgs/Tdoa";
  }

  static const char* value(const ::cf_msgs::Tdoa_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::cf_msgs::Tdoa_<ContainerAllocator> >
{
  static const char* value()
  {
    return "std_msgs/Header header\n"
"int32 idA\n"
"int32 idB\n"
"float32 data\n"
"================================================================================\n"
"MSG: std_msgs/Header\n"
"# Standard metadata for higher-level stamped data types.\n"
"# This is generally used to communicate timestamped data \n"
"# in a particular coordinate frame.\n"
"# \n"
"# sequence ID: consecutively increasing ID \n"
"uint32 seq\n"
"#Two-integer timestamp that is expressed as:\n"
"# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')\n"
"# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')\n"
"# time-handling sugar is provided by the client library\n"
"time stamp\n"
"#Frame this data is associated with\n"
"string frame_id\n"
;
  }

  static const char* value(const ::cf_msgs::Tdoa_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::cf_msgs::Tdoa_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.header);
      stream.next(m.idA);
      stream.next(m.idB);
      stream.next(m.data);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct Tdoa_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::cf_msgs::Tdoa_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::cf_msgs::Tdoa_<ContainerAllocator>& v)
  {
    s << indent << "header: ";
    s << std::endl;
    Printer< ::std_msgs::Header_<ContainerAllocator> >::stream(s, indent + "  ", v.header);
    s << indent << "idA: ";
    Printer<int32_t>::stream(s, indent + "  ", v.idA);
    s << indent << "idB: ";
    Printer<int32_t>::stream(s, indent + "  ", v.idB);
    s << indent << "data: ";
    Printer<float>::stream(s, indent + "  ", v.data);
  }
};

} // namespace message_operations
} // namespace ros

#endif // CF_MSGS_MESSAGE_TDOA_H
