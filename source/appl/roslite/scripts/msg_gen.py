#!/usr/bin/env python

import sys
import os
sys.path.append(os.path.dirname(os.path.abspath(__file__)) + '/ros_packages')
import traceback

import roslib.msgs 
import roslib.packages
import roslib.gentools
from rospkg import RosPack
try:
    set
except NameError:
    from sets import Set as set

try:
    from cStringIO import StringIO #Python 2.x
except ImportError:
    from io import StringIO #Python 3.x

import commons  # local module

MSG_TYPE_TO_CPP = {'byte': 'int8_t', 'char': 'uint8_t',
                   'bool': 'uint8_t',
                   'uint8': 'uint8_t', 'int8': 'int8_t', 
                   'uint16': 'uint16_t', 'int16': 'int16_t', 
                   'uint32': 'uint32_t', 'int32': 'int32_t',
                   'uint64': 'uint64_t', 'int64': 'int64_t',
                   'float32': 'float',
                   'float64': 'double',
                   'string': 'std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other > ',
                   'time': 'ROSLITE_NAMESPACE::Time',
                   'duration': 'ROSLITE_NAMESPACE::Duration'}

def msg_type_to_cpp(type):
    """
    Converts a message type (e.g. uint32, std_msgs/String, etc.) into the C++ declaration
    for that type (e.g. uint32_t, std_msgs::String_<ContainerAllocator>)
    
    @param type: The message type
    @type type: str
    @return: The C++ declaration
    @rtype: str
    """
    (base_type, is_array, array_len) = roslib.msgs.parse_type(type)
    cpp_type = None
    if (roslib.msgs.is_builtin(base_type)):
        cpp_type = MSG_TYPE_TO_CPP[base_type]
    elif (len(base_type.split('/')) == 1):
        if (roslib.msgs.is_header_type(base_type)):
            cpp_type = ' ::NAMESPACE_std_msgs::Header_<ContainerAllocator> '
        else:
            cpp_type = '%s_<ContainerAllocator> '%(base_type)
    else:
        pkg = base_type.split('/')[0]
        msg = base_type.split('/')[1]
        cpp_type = ' ::NAMESPACE_%s::%s_<ContainerAllocator> '%(pkg, msg)
        
    if (is_array):
        if (array_len is None):
            return 'std::vector<%s, typename ContainerAllocator::template rebind<%s>::other > '%(cpp_type, cpp_type)
        else:
            return 'boost::array<%s, %s> '%(cpp_type, array_len)
    else:
        return cpp_type

def is_time_or_duration(type):
    return (type == 'time' or type == 'duration')

def cpp_message_declarations(name_prefix, msg):
    """
    Returns the different possible C++ declarations for a message given the message itself.
    
    @param name_prefix: The C++ prefix to be prepended to the name, e.g. "std_msgs::"
    @type name_prefix: str
    @param msg: The message type
    @type msg: str
    @return: A tuple of 3 different names.  cpp_message_decelarations("std_msgs::", "String") returns the tuple
        ("std_msgs::String_", "std_msgs::String_<ContainerAllocator>", "std_msgs::String")
    @rtype: str 
    """
    pkg, basetype = roslib.names.package_resource_name(msg)
    cpp_name = ' ::%s%s'%(name_prefix, msg)
    if (pkg):
        cpp_name = ' ::%s::%s'%(pkg, basetype)
    return ('%s_'%(cpp_name), '%s_<ContainerAllocator> '%(cpp_name), '%s'%(cpp_name))

def write_begin(s, spec, file):
    """
    Writes the beginning of the header file: a comment saying it's auto-generated and the include guards
    
    @param s: The stream to write to
    @type s: stream
    @param spec: The spec
    @type spec: roslib.msgs.MsgSpec
    @param file: The file this message is being generated for
    @type file: str
    """
    s.write('#ifndef ROSLITE_%s_MESSAGE_%s_H\n'%(spec.package.upper(), spec.short_name.upper()))
    s.write('#define ROSLITE_%s_MESSAGE_%s_H\n'%(spec.package.upper(), spec.short_name.upper()))

def write_end(s, spec):
    """
    Writes the end of the header file: the ending of the include guards
    
    @param s: The stream to write to
    @type s: stream
    @param spec: The spec
    @type spec: roslib.msgs.MsgSpec
    """
    s.write('#endif // ROSLITE_%s_MESSAGE_%s_H\n'%(spec.package.upper(), spec.short_name.upper()))
        
def write_generic_includes(s):
    """
    Writes the includes that all messages need
    
    @param s: The stream to write to
    @type s: stream
    """
    s.write('\n')
    s.write('#include <string>\n')
    s.write('#include <vector>\n')
    s.write('#include <map>\n')
    s.write('#include <ostream>\n\n')
    # s.write('#include "ros/serialization.h"\n')
    # s.write('#include "ros/builtin_message_traits.h"\n')
    # s.write('#include "ros/message_operations.h"\n')
    # s.write('#include "ros/time.h"\n\n')
    # s.write('#include "ros/macros.h"\n\n')
    # s.write('#include "ros/assert.h"\n\n')

def write_includes(s, package, spec):
    s.write('#if ROSLITE_TARGET_CLUSTER_ID == 0\n')
    s.write('  #include "roslite/include/ros/common.h"\n')
    s.write('  #include "roslite/include/ros/serialization.h"\n')
    s.write('  #include "%s/%s.h"\n' % (package, spec.short_name))
    for field in spec.parsed_fields():
        if (not field.is_builtin):
            if (field.is_header):
                s.write('  #include "roslite/include/std_msgs/Header.h"\n')
            else:
                (pkg, name) = roslib.names.package_resource_name(field.base_type)
                pkg = pkg or spec.package # convert '' to package
                s.write('  #include "roslite/include/%s/%s.h"\n'%(pkg, name))
    s.write('#else\n')
    s.write('  #include "ros/common.h"\n')
    s.write('  #include "ros/serialization.h"\n')
    for field in spec.parsed_fields():
        if (not field.is_builtin):
            if (field.is_header):
                s.write('  #include "std_msgs/Header.h"\n')
            else:
                (pkg, name) = roslib.names.package_resource_name(field.base_type)
                pkg = pkg or spec.package # convert '' to package
                s.write('  #include "%s/%s.h"\n'%(pkg, name))
    s.write('#endif\n')
    s.write('\n') 

def write_struct(s, package, spec, cpp_name_prefix, extra_deprecated_traits = {}):
    """
    Writes the entire message struct: declaration, constructors, members, constants and (deprecated) member functions
    @param s: The stream to write to
    @type s: stream
    @param spec: The message spec
    @type spec: roslib.msgs.MsgSpec
    @param cpp_name_prefix: The C++ prefix to use when referring to the message, e.g. "std_msgs::"
    @type cpp_name_prefix: str
    """
    
    msg = spec.short_name
    s.write('template <class ContainerAllocator>\n')
    s.write('struct %s_ {\n'%(msg))
    s.write('  typedef %s_<ContainerAllocator> Type;\n\n'%(msg))
    
    write_constructors(s, spec, cpp_name_prefix)
    write_members(s, spec)
    write_constant_declarations(s, spec)  # ??
    
    #rospack = RosPack()
    #gendeps_dict = roslib.gentools.get_dependencies(spec, spec.package, compute_files=False, rospack=rospack)
    #md5sum = roslib.gentools.compute_md5(gendeps_dict, rospack=rospack)
    #full_text = compute_full_text_escaped(gendeps_dict)
    
    # write_deprecated_member_functions(s, spec, dict(list({'MD5Sum': md5sum, 'DataType': '%s/%s'%(spec.package, spec.short_name), 'MessageDefinition': full_text}.items()) + list(extra_deprecated_traits.items())))
    
    (cpp_msg_unqualified, cpp_msg_with_alloc, cpp_msg_base) = cpp_message_declarations(cpp_name_prefix, msg)
    # s.write('  typedef boost::shared_ptr<%s> Ptr;\n'%(cpp_msg_with_alloc))  # MOD
    s.write('  typedef ::std::shared_ptr<%s> Ptr;\n'%(cpp_msg_with_alloc))  # MOD
    # s.write('  typedef boost::shared_ptr<%s const> ConstPtr;\n'%(cpp_msg_with_alloc))  # MOD
    s.write('  typedef ::std::shared_ptr<%s const> ConstPtr;\n'%(cpp_msg_with_alloc))  # MOD

    s.write('\n')

    s.write('#if ROSLITE_TARGET_CLUSTER_ID == 0\n')
    s.write('  static Type FromRosMsg(const typename %s::%s_<ContainerAllocator>& ros_msg) {\n' % (package, msg))
    s.write('    Type roslite_msg;\n')
    for m in spec.parsed_fields():
        (base_type, is_array, array_len) = roslib.msgs.parse_type(m.type)
        if (roslib.msgs.is_builtin(base_type) and not is_time_or_duration(base_type)):
            s.write('    roslite_msg.%s = ros_msg.%s;\n' % (m.name, m.name))
        elif (is_array):
            s.write('    for (int i = 0; i < (int)ros_msg.%s.size(); ++i) {\n' % (m.name))
            s.write('      roslite_msg.%s.push_back(%s::FromRosMsg(ros_msg.%s[i]));\n' % (m.name, msg_type_to_cpp(base_type), m.name))
            s.write('    }\n')
        else:
            s.write('    roslite_msg.%s = %s::FromRosMsg(ros_msg.%s);\n' % (m.name, msg_type_to_cpp(m.type), m.name))
    s.write('    return roslite_msg;\n')
    s.write('  }\n')
    s.write('\n')
    s.write('  typename %s::%s_<ContainerAllocator> ToRosMsg() const {\n' % (package, msg))
    s.write('    typename %s::%s_<ContainerAllocator> ros_msg;\n' % (package, msg))
    for m in spec.parsed_fields():
        (base_type, is_array, array_len) = roslib.msgs.parse_type(m.type)
        if (roslib.msgs.is_builtin(base_type) and not is_time_or_duration(base_type)):
            s.write('    ros_msg.%s = %s;\n' % (m.name, m.name))
        elif (is_array):
            s.write('    for (int i = 0; i < (int)%s.size(); ++i) {\n' % (m.name))
            s.write('      ros_msg.%s.push_back(%s[i].ToRosMsg());\n' % (m.name, m.name))
            s.write('    }\n')
        else:
            s.write('    ros_msg.%s = %s.ToRosMsg();\n' % (m.name, m.name))
    s.write('    return ros_msg;\n')
    s.write('  }\n')
    s.write('  \n')
    
    s.write('  static typename Type::ConstPtr FromRosMsgPtr(const typename %s::%s_<ContainerAllocator>::ConstPtr& ros_msg) {\n' % (package, msg))
    s.write('    typename Type::Ptr roslite_msg(new Type());\n')
    s.write('    *roslite_msg = FromRosMsg(*ros_msg);\n')
    s.write('    return roslite_msg;\n')
    s.write('  }\n')
    s.write('\n')
    s.write('  typename %s::%s_<ContainerAllocator>::ConstPtr ToRosMsgPtr() const {\n' % (package, msg))
    s.write('    typename %s::%s_<ContainerAllocator>::Ptr ros_msg(new %s::%s_<ContainerAllocator>());\n' % (package, msg, package, msg))
    s.write('    *ros_msg = ToRosMsg();\n')
    s.write('    return ros_msg;\n')
    s.write('  }\n')
    s.write('#endif\n')
    
    s.write('\n')

    s.write('}; // struct %s\n'%(msg))
    
    s.write('typedef %s_<std::allocator<void> > %s;\n\n'%(cpp_msg_base, msg))
    # s.write('typedef boost::shared_ptr<%s> %sPtr;\n'%(cpp_msg_base, msg))  # MOD
    s.write('typedef ::std::shared_ptr<%s> %sPtr;\n'%(cpp_msg_base, msg))  # MOD
    # s.write('typedef boost::shared_ptr<%s const> %sConstPtr;\n\n'%(cpp_msg_base, msg))  # MOD
    s.write('typedef ::std::shared_ptr<%s const> %sConstPtr;\n\n'%(cpp_msg_base, msg))  # MOD

def default_value(type):
    """
    Returns the value to initialize a message member with.  0 for integer types, 0.0 for floating point, false for bool,
    empty string for everything else
    
    @param type: The type
    @type type: str
    """
    if type in ['byte', 'int8', 'int16', 'int32', 'int64',
                'char', 'uint8', 'uint16', 'uint32', 'uint64']:
        return '0'
    elif type in ['float32', 'float64']:
        return '0.0'
    elif type == 'bool':
        return 'false'
        
    return ""

def takes_allocator(type):
    """
    Returns whether or not a type can take an allocator in its constructor.  False for all builtin types except string.
    True for all others.
    
    @param type: The type
    @type: str
    """
    return not type in ['byte', 'int8', 'int16', 'int32', 'int64',
                        'char', 'uint8', 'uint16', 'uint32', 'uint64',
                        'float32', 'float64', 'bool', 'time', 'duration']

def write_initializer_list(s, spec, container_gets_allocator):
    """
    Writes the initializer list for a constructor
    
    @param s: The stream to write to
    @type s: stream
    @param spec: The message spec
    @type spec: roslib.msgs.MsgSpec
    @param container_gets_allocator: Whether or not a container type (whether it's another message, a vector, array or string)
        should have the allocator passed to its constructor.  Assumes the allocator is named _alloc.
    @type container_gets_allocator: bool
    """
    
    i = 0
    for field in spec.parsed_fields():
        if (i == 0):
            s.write('  : ')
        else:
            s.write('  , ')
            
        val = default_value(field.base_type)
        use_alloc = takes_allocator(field.base_type)
        if (field.is_array):
            if (field.array_len is None and container_gets_allocator):
                s.write('%s(_alloc)\n'%(field.name))
            else:
                s.write('%s()\n'%(field.name))
        else:
            if (container_gets_allocator and use_alloc):
                s.write('%s(_alloc)\n'%(field.name))
            else:
                s.write('%s(%s)\n'%(field.name, val))
        i = i + 1

def write_fixed_length_assigns(s, spec, container_gets_allocator, cpp_name_prefix):
    """
    Initialize any fixed-length arrays
    
    @param s: The stream to write to
    @type s: stream
    @param spec: The message spec
    @type spec: roslib.msgs.MsgSpec
    @param container_gets_allocator: Whether or not a container type (whether it's another message, a vector, array or string)
        should have the allocator passed to its constructor.  Assumes the allocator is named _alloc.
    @type container_gets_allocator: bool
    @param cpp_name_prefix: The C++ prefix to use when referring to the message, e.g. "std_msgs::"
    @type cpp_name_prefix: str
    """
    # Assign all fixed-length arrays their default values
    for field in spec.parsed_fields():
        if (not field.is_array or field.array_len is None):
            continue
        
        val = default_value(field.base_type)
        if (container_gets_allocator and takes_allocator(field.base_type)):
            # String is a special case, as it is the only builtin type that takes an allocator
            if (field.base_type == "string"):
                string_cpp = msg_type_to_cpp("string")
                s.write('    %s.assign(%s(_alloc));\n'%(field.name, string_cpp))
            else:
                (cpp_msg_unqualified, cpp_msg_with_alloc, _) = cpp_message_declarations(cpp_name_prefix, field.base_type)
                s.write('    %s.assign(%s(_alloc));\n'%(field.name, cpp_msg_with_alloc))
        elif (len(val) > 0):
            s.write('    %s.assign(%s);\n'%(field.name, val))

def write_constructors(s, spec, cpp_name_prefix):
    """
    Writes any necessary constructors for the message
    
    @param s: The stream to write to
    @type s: stream
    @param spec: The message spec
    @type spec: roslib.msgs.MsgSpec
    @param cpp_name_prefix: The C++ prefix to use when referring to the message, e.g. "std_msgs::"
    @type cpp_name_prefix: str
    """
    
    msg = spec.short_name
    
    # Default constructor
    s.write('  %s_()\n'%(msg))
    write_initializer_list(s, spec, False)
    s.write('  {\n')
    write_fixed_length_assigns(s, spec, False, cpp_name_prefix)
    s.write('  }\n\n')
    
    # Constructor that takes an allocator constructor
    s.write('  %s_(const ContainerAllocator& _alloc)\n'%(msg))
    write_initializer_list(s, spec, True)
    s.write('  {\n')
    write_fixed_length_assigns(s, spec, True, cpp_name_prefix)
    s.write('  }\n\n')

def write_member(s, field):
    """
    Writes a single member's declaration and type typedef
    
    @param s: The stream to write to
    @type s: stream
    @param type: The member type
    @type type: str
    @param name: The name of the member
    @type name: str
    """
    cpp_type = msg_type_to_cpp(field.type)
    s.write('  typedef %s _%s_type;\n'%(cpp_type, field.name))
    s.write('  %s %s;\n\n'%(cpp_type, field.name))

def write_members(s, spec):
    """
    Write all the member declarations
    
    @param s: The stream to write to
    @type s: stream
    @param spec: The message spec
    @type spec: roslib.msgs.MsgSpec
    """
    [write_member(s, field) for field in spec.parsed_fields()]

def write_constant_declaration(s, constant):
    """
    Write a constant value as a static member
    
    @param s: The stream to write to
    @type s: stream
    @param constant: The constant
    @type constant: roslib.msgs.Constant
    """
    
    # integral types get their declarations as enums to allow use at compile time
    if (constant.type in ['byte', 'int8', 'int16', 'int32', 'int64', 'char', 'uint8', 'uint16', 'uint32', 'uint64']):
        s.write('  enum { %s = %s };\n'%(constant.name, constant.val))
    else:
        s.write('  static const %s %s;\n'%(msg_type_to_cpp(constant.type), constant.name))

def write_constant_declarations(s, spec):
    """
    Write all the constants from a spec as static members
    
    @param s: The stream to write to
    @type s: stream
    @param spec: The message spec
    @type spec: roslib.msgs.MsgSpec
    """
    [write_constant_declaration(s, constant) for constant in spec.constants]
    s.write('\n')

def write_constant_definition(s, spec, constant):
    """
    Write a constant value as a static member
    
    @param s: The stream to write to
    @type s: stream
    @param constant: The constant
    @type constant: roslib.msgs.Constant
    """
    
    # integral types do not need a definition, since they've been defined where they are declared
    if (constant.type not in ['byte', 'int8', 'int16', 'int32', 'int64', 'char', 'uint8', 'uint16', 'uint32', 'uint64', 'string']):
        s.write('template<typename ContainerAllocator> const %s %s_<ContainerAllocator>::%s = %s;\n'%(msg_type_to_cpp(constant.type), spec.short_name, constant.name, constant.val))
    elif (constant.type == 'string'):
        s.write('template<typename ContainerAllocator> const %s %s_<ContainerAllocator>::%s = "%s";\n'%(msg_type_to_cpp(constant.type), spec.short_name, constant.name, escape_string(constant.val)))
        
def write_constant_definitions(s, spec):
    """
    Write all the constants from a spec as static members
    
    @param s: The stream to write to
    @type s: stream
    @param spec: The message spec
    @type spec: roslib.msgs.MsgSpec
    """
    [write_constant_definition(s, spec, constant) for constant in spec.constants]
    s.write('\n')
        
def write_serialization(s, spec, cpp_name_prefix):
    """
    Writes the Serializer class for a message
    
    @param s: Stream to write to
    @type s: stream
    @param spec: The message spec
    @type spec: roslib.msgs.MsgSpec
    @param cpp_name_prefix: The C++ prefix to prepend to a message to refer to it (e.g. "std_msgs::")
    @type cpp_name_prefix: str
    """
    (cpp_msg_unqualified, cpp_msg_with_alloc, _) = cpp_message_declarations(cpp_name_prefix, spec.short_name)
    
    s.write('namespace ROSLITE_NAMESPACE\n{\n')
    s.write('namespace serialization\n{\n\n')
    
    s.write('template<class ContainerAllocator> struct Serializer<%s>\n{\n'%(cpp_msg_with_alloc))
    
    s.write('  template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)\n  {\n')
    for field in spec.parsed_fields():
        s.write('    stream.next(m.%s);\n'%(field.name))
    s.write('  }\n\n')
    
    s.write('  ROS_DECLARE_ALLINONE_SERIALIZER;\n')
    
    s.write('}; // struct %s_\n'%(spec.short_name))
        
    s.write('} // namespace serialization\n')
    s.write('} // namespace ROSLITE_NAMESPACE\n\n')
 
def generate(msg_path):
    """
    Generate a message
    
    @param msg_path: The path to the .msg file
    @type msg_path: str
    """
    # (package_dir, package) = roslib.packages.get_dir_pkg(msg_path)  # TODO
    # package = os.path.dirname(msg_path)  # TODO
    splited = os.path.dirname(msg_path).rsplit('/',1)
    package = splited[1]
    # print package

    (_, spec) = roslib.msgs.load_from_file(msg_path, package)
        
    s = StringIO()
    # s.write('// [test] First line.\n')
    commons.write_note(s, os.path.relpath(msg_path))
    write_begin(s, spec, msg_path)
    write_generic_includes(s)
    write_includes(s, package, spec)

    namespace_macro = 'NAMESPACE_' + package;
    cpp_prefix = namespace_macro + '::'

    all_pkgs = set();
    for field in spec.parsed_fields():
        if (not field.is_builtin):
            if (field.is_header):
                all_pkgs.add('std_msgs')
            else:
                (pkg, name) = roslib.names.package_resource_name(field.base_type)
                pkg = pkg or spec.package # convert '' to package
                all_pkgs.add(pkg)
    
    s.write('#if ROSLITE_TARGET_CLUSTER_ID == 0\n')
    s.write('  #define %s roslite_%s\n' % (namespace_macro, package))
    for pkg in all_pkgs:
        s.write('  #define NAMESPACE_%s roslite_%s\n' % (pkg, pkg))
    s.write('#else\n')
    s.write('  #define %s %s\n' % (namespace_macro, package))
    for pkg in all_pkgs:
        s.write('  #define NAMESPACE_%s %s\n' % (pkg, pkg))
    s.write('#endif\n')
    s.write('\n')
    s.write('namespace %s\n{\n' % (namespace_macro))
    write_struct(s, package, spec, cpp_prefix)
    write_constant_definitions(s, spec)  # ??
    # write_ostream_operator(s, spec, cpp_prefix)  # MOD
    s.write('} // namespace NAMESPACE_%s\n\n'%(package))
    
    # rospack = RosPack()  # MOD
    # write_traits(s, spec, cpp_prefix, rospack=rospack)  # MOD
    write_serialization(s, spec, cpp_prefix)
    # write_operations(s, spec, cpp_prefix)  # MOD

    s.write('#undef %s\n' % (namespace_macro))
    s.write('\n')
    
    # MOD
    # # HACK HACK HACK.  The moving of roslib/Header causes many problems.  We end up having to make roslib/Header act exactly
    # # like std_msgs/Header (as in, constructor that takes it, as well as operator std_msgs::Header()), and it needs to be
    # # available wherever std_msgs/Header.h has been included
    # if (package == "std_msgs" and spec.short_name == "Header"):
    #     s.write("#define STD_MSGS_INCLUDING_HEADER_DEPRECATED_DEF 1\n")
    #     s.write("#include <std_msgs/header_deprecated_def.h>\n")
    #     s.write("#undef STD_MSGS_INCLUDING_HEADER_DEPRECATED_DEF\n\n") 
    
    write_end(s, spec)
        
    # output_dir = '%s/msg_gen/cpp/include/%s'%(package_dir, package)  # MOD
    output_dir = os.path.dirname(os.path.abspath(__file__)) + '/../../roslite/include/%s'%(package)  # MOD

    commons.print_generated_file(package + '/' + spec.short_name + '.h' )

    if (not os.path.exists(output_dir)):
        # if we're being run concurrently, the above test can report false but os.makedirs can still fail if
        # another copy just created the directory
        try:
            os.makedirs(output_dir)
        except OSError as e:
            pass
         
    f = open('%s/%s.h'%(output_dir, spec.short_name), 'w')
    f.write(s.getvalue() + "\n")
    
    s.close()

def generate_messages(argv):
    for arg in argv[1:]:
        generate(arg)

def test():
    print('--- Generating message header file (.h) in ./generated_files from .msg ---')
    print('--- $ python gen_msg.py [.msg path] [.msg path] ... ---\n')

# start from here
if __name__ == '__main__':
    # test()
    generate_messages(sys.argv)
    # print('\n---  Completed!! ---')
