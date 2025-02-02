// Generated by gencpp from file visual_processing/SetParamRequest.msg
// DO NOT EDIT!


#ifndef VISUAL_PROCESSING_MESSAGE_SETPARAMREQUEST_H
#define VISUAL_PROCESSING_MESSAGE_SETPARAMREQUEST_H


#include <string>
#include <vector>
#include <map>

#include <ros/types.h>
#include <ros/serialization.h>
#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>


namespace visual_processing
{
template <class ContainerAllocator>
struct SetParamRequest_
{
  typedef SetParamRequest_<ContainerAllocator> Type;

  SetParamRequest_()
    : type()
    , color()  {
    }
  SetParamRequest_(const ContainerAllocator& _alloc)
    : type(_alloc)
    , color(_alloc)  {
  (void)_alloc;
    }



   typedef std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other >  _type_type;
  _type_type type;

   typedef std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other >  _color_type;
  _color_type color;





  typedef boost::shared_ptr< ::visual_processing::SetParamRequest_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::visual_processing::SetParamRequest_<ContainerAllocator> const> ConstPtr;

}; // struct SetParamRequest_

typedef ::visual_processing::SetParamRequest_<std::allocator<void> > SetParamRequest;

typedef boost::shared_ptr< ::visual_processing::SetParamRequest > SetParamRequestPtr;
typedef boost::shared_ptr< ::visual_processing::SetParamRequest const> SetParamRequestConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::visual_processing::SetParamRequest_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::visual_processing::SetParamRequest_<ContainerAllocator> >::stream(s, "", v);
return s;
}


template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator==(const ::visual_processing::SetParamRequest_<ContainerAllocator1> & lhs, const ::visual_processing::SetParamRequest_<ContainerAllocator2> & rhs)
{
  return lhs.type == rhs.type &&
    lhs.color == rhs.color;
}

template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator!=(const ::visual_processing::SetParamRequest_<ContainerAllocator1> & lhs, const ::visual_processing::SetParamRequest_<ContainerAllocator2> & rhs)
{
  return !(lhs == rhs);
}


} // namespace visual_processing

namespace ros
{
namespace message_traits
{





template <class ContainerAllocator>
struct IsFixedSize< ::visual_processing::SetParamRequest_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::visual_processing::SetParamRequest_<ContainerAllocator> const>
  : FalseType
  { };

template <class ContainerAllocator>
struct IsMessage< ::visual_processing::SetParamRequest_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::visual_processing::SetParamRequest_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::visual_processing::SetParamRequest_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::visual_processing::SetParamRequest_<ContainerAllocator> const>
  : FalseType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::visual_processing::SetParamRequest_<ContainerAllocator> >
{
  static const char* value()
  {
    return "e33fd06afe7da7ba7e11dfc0bf097031";
  }

  static const char* value(const ::visual_processing::SetParamRequest_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0xe33fd06afe7da7baULL;
  static const uint64_t static_value2 = 0x7e11dfc0bf097031ULL;
};

template<class ContainerAllocator>
struct DataType< ::visual_processing::SetParamRequest_<ContainerAllocator> >
{
  static const char* value()
  {
    return "visual_processing/SetParamRequest";
  }

  static const char* value(const ::visual_processing::SetParamRequest_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::visual_processing::SetParamRequest_<ContainerAllocator> >
{
  static const char* value()
  {
    return "string type\n"
"string color\n"
;
  }

  static const char* value(const ::visual_processing::SetParamRequest_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::visual_processing::SetParamRequest_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.type);
      stream.next(m.color);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct SetParamRequest_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::visual_processing::SetParamRequest_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::visual_processing::SetParamRequest_<ContainerAllocator>& v)
  {
    s << indent << "type: ";
    Printer<std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other > >::stream(s, indent + "  ", v.type);
    s << indent << "color: ";
    Printer<std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other > >::stream(s, indent + "  ", v.color);
  }
};

} // namespace message_operations
} // namespace ros

#endif // VISUAL_PROCESSING_MESSAGE_SETPARAMREQUEST_H
