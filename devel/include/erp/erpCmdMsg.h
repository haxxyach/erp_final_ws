// Generated by gencpp from file erp/erpCmdMsg.msg
// DO NOT EDIT!


#ifndef ERP_MESSAGE_ERPCMDMSG_H
#define ERP_MESSAGE_ERPCMDMSG_H


#include <string>
#include <vector>
#include <memory>

#include <ros/types.h>
#include <ros/serialization.h>
#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>


namespace erp
{
template <class ContainerAllocator>
struct erpCmdMsg_
{
  typedef erpCmdMsg_<ContainerAllocator> Type;

  erpCmdMsg_()
    : e_stop(false)
    , gear(0)
    , speed(0)
    , steer(0)
    , brake(0)  {
    }
  erpCmdMsg_(const ContainerAllocator& _alloc)
    : e_stop(false)
    , gear(0)
    , speed(0)
    , steer(0)
    , brake(0)  {
  (void)_alloc;
    }



   typedef uint8_t _e_stop_type;
  _e_stop_type e_stop;

   typedef uint8_t _gear_type;
  _gear_type gear;

   typedef uint8_t _speed_type;
  _speed_type speed;

   typedef int32_t _steer_type;
  _steer_type steer;

   typedef uint8_t _brake_type;
  _brake_type brake;





  typedef boost::shared_ptr< ::erp::erpCmdMsg_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::erp::erpCmdMsg_<ContainerAllocator> const> ConstPtr;

}; // struct erpCmdMsg_

typedef ::erp::erpCmdMsg_<std::allocator<void> > erpCmdMsg;

typedef boost::shared_ptr< ::erp::erpCmdMsg > erpCmdMsgPtr;
typedef boost::shared_ptr< ::erp::erpCmdMsg const> erpCmdMsgConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::erp::erpCmdMsg_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::erp::erpCmdMsg_<ContainerAllocator> >::stream(s, "", v);
return s;
}


template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator==(const ::erp::erpCmdMsg_<ContainerAllocator1> & lhs, const ::erp::erpCmdMsg_<ContainerAllocator2> & rhs)
{
  return lhs.e_stop == rhs.e_stop &&
    lhs.gear == rhs.gear &&
    lhs.speed == rhs.speed &&
    lhs.steer == rhs.steer &&
    lhs.brake == rhs.brake;
}

template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator!=(const ::erp::erpCmdMsg_<ContainerAllocator1> & lhs, const ::erp::erpCmdMsg_<ContainerAllocator2> & rhs)
{
  return !(lhs == rhs);
}


} // namespace erp

namespace ros
{
namespace message_traits
{





template <class ContainerAllocator>
struct IsMessage< ::erp::erpCmdMsg_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::erp::erpCmdMsg_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::erp::erpCmdMsg_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::erp::erpCmdMsg_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::erp::erpCmdMsg_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::erp::erpCmdMsg_<ContainerAllocator> const>
  : FalseType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::erp::erpCmdMsg_<ContainerAllocator> >
{
  static const char* value()
  {
    return "6c8d779558341c7cf957ab6058219fbb";
  }

  static const char* value(const ::erp::erpCmdMsg_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0x6c8d779558341c7cULL;
  static const uint64_t static_value2 = 0xf957ab6058219fbbULL;
};

template<class ContainerAllocator>
struct DataType< ::erp::erpCmdMsg_<ContainerAllocator> >
{
  static const char* value()
  {
    return "erp/erpCmdMsg";
  }

  static const char* value(const ::erp::erpCmdMsg_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::erp::erpCmdMsg_<ContainerAllocator> >
{
  static const char* value()
  {
    return "bool e_stop\n"
"uint8 gear\n"
"uint8 speed\n"
"int32 steer\n"
"uint8 brake\n"
;
  }

  static const char* value(const ::erp::erpCmdMsg_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::erp::erpCmdMsg_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.e_stop);
      stream.next(m.gear);
      stream.next(m.speed);
      stream.next(m.steer);
      stream.next(m.brake);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct erpCmdMsg_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::erp::erpCmdMsg_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::erp::erpCmdMsg_<ContainerAllocator>& v)
  {
    s << indent << "e_stop: ";
    Printer<uint8_t>::stream(s, indent + "  ", v.e_stop);
    s << indent << "gear: ";
    Printer<uint8_t>::stream(s, indent + "  ", v.gear);
    s << indent << "speed: ";
    Printer<uint8_t>::stream(s, indent + "  ", v.speed);
    s << indent << "steer: ";
    Printer<int32_t>::stream(s, indent + "  ", v.steer);
    s << indent << "brake: ";
    Printer<uint8_t>::stream(s, indent + "  ", v.brake);
  }
};

} // namespace message_operations
} // namespace ros

#endif // ERP_MESSAGE_ERPCMDMSG_H