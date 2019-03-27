#!/usr/bin/env python

import sys
import os
import re
import glob

from argparse import ArgumentParser

try:
    from cStringIO import StringIO #Python 2.x
except ImportError:
    from io import StringIO #Python 3.x

COMMENT_1_LINE         = '//'
COMMENT_START          = '/*'
COMMENT_END            = '*/'

FUNC_ROS_INIT          = 'ros::init'
FUNC_INIT              = 'init'

FUNC_ADVERTISE_1       = '.advertise'
FUNC_ADVERTISE_2       = '->advertise'

FUNC_SUBSCRIBE_1       = '.subscribe'
FUNC_SUBSCRIBE_2       = '->subscribe'

PATTERN_SUBSCRIBE_1    = r"[a-zA-Z0-9_:]*::ConstPtr"
PATTERN_SUBSCRIBE_2    = r"MessageEvent<[a-zA-Z0-9_:]*\s+const>"
PATTERN_SUBSCRIBE_3    = r"const\s+[a-zA-Z0-9_:]*ConstPtr&"
PATTERN_SUBSCRIBE_4    = r"[a-zA-Z0-9_:]*ConstPtr"
PATTERN_SUBSCRIBE_5    = r"const\s+[a-zA-Z0-9_:]*&"

LEFT_BRACKET           = '('
RIGHT_BRACKET          = ')'
TEMPLATE_LEFT_BRACKET  = '<'
TEMPLATE_RIGHT_BRACKET = '>'
COMMA                  = ','
HYPHEN                 = '-'
DOUBLE_COLON           = '::'
ARROW                  = '->'
PERIOD                 = '.'
EQUAL                  = '='

DEFINITION_MACRO       = '#define'

usage                  = "rosl_create_map file1,file2,...,filen -o output_file [-c from-to]"

class MapInformation:
    name = ""
    cluster_number = 0
    cpu_number = 0
    publish = []
    publish_type = []
    subscribe = []
    subscribe_type = []

    def __init__(self, name, cluster_number, cpu_number, publish, publish_type, subscribe, subscribe_type):
        self.name = name
        self.cluster_number = cluster_number
        self.cpu_number = cpu_number
        self.publish = publish
        self.publish_type = publish_type
        self.subscribe = subscribe
        self.subscribe_type = subscribe_type

    def write_map(self, stream):
        stream.write("- name: " )
        write_str_data(self.name, stream)
        if self.cluster_number == 0:
            stream.write("  cluster: \n")
        else:
            stream.write("  cluster: " + str(self.cluster_number) + "\n")
        if self.cpu_number is None:
            stream.write("  cpu: \n")
        else:
            stream.write("  cpu: " + str(self.cpu_number) + "\n")
        stream.write("  publish: ")
        write_list_data(self.publish, stream)
        stream.write("  publish_type: ")
        write_list_data_with_double_quate(self.publish_type, stream)
        stream.write("  subscribe: ")
        write_list_data(self.subscribe, stream)
        stream.write("  subscribe_type: ")
        write_list_data_with_double_quate(self.subscribe_type, stream)

    # for debug print
    def print_map(self):
        print("- name: " + self.name)
        print("  cluster: " + str(self.cluster_number))
        print("  cpu: " + str(self.cpu_number))
        print("  publish: " + str(self.publish))
        print("  publish_type: " + str(self.publish_type))
        print("  subscribe: " + str(self.subscribe))
        print("  subscribe_type: " + str(self.subscribe_type))

def write_str_data(str, stream):
    result = ""
    result = (re.sub(r"\'|\"", "", str)) + "\n"

    stream.write(result)

def write_list_data(list, stream):
    str = ""
    str += "["
    if len(list) is not 0:
        for data in list:
            str += (re.sub(r"\'|\"", "", data) + ", ")
        str = str[:-2]
    str += "]\n"

    stream.write(str)

def write_list_data_with_double_quate(list, stream):
    str = ""
    str += "["
    if len(list) is not 0:
        for data in list:
            str += "\"" + (re.sub(r"\'|\"", "", data) + "\", ")
        str = str[:-2]
    str += "]\n"

    stream.write(str)

def search_macro(macro, lines):
    # search "#define macro value" in whole file
    result = ""
    for index,line in enumerate(lines):
        m = re.search((DEFINITION_MACRO) + r"\s+" + (macro) + r"\s+", line)
        if m is not None:
            result = line[m.end():].strip()
            break

    return result


def search_variable(variable, lines):
    # search variable in whole file
    result = ""
    for index,line in enumerate(lines):
        m = re.search((variable) + r"\s*" + (EQUAL) + r"\s*", line)
        if m is not None:
            result = line[m.end()+1:].strip()
            # delete last semi-colon
            result = result[:-1]
            break

    return result

def get_value(arg, lines):
    m = re.search("\"" + "[a-zA-Z0-9_]+" + "\"", arg)
    if m is not None:
        return arg
    else:
        result = search_macro(arg, lines)
        if result != "":
            return result
        result = search_variable(arg, lines)
        if result != "":
            return result
    return arg

def delete_comment(lines):
    del_multiple_lines = False
    for index, line in enumerate(lines):
        if del_multiple_lines is True:
            location = line.find(COMMENT_END)
            if location == -1:
                lines[index] = ""
            else:
                lines[index] = line[location+2:]
                del_multiple_lines = False
        location = line.find(COMMENT_START)
        if location > -1:
            location2 = line.find(COMMENT_END)
            if location == 0 and location2 > -1:
                lines[index] = line[location2+2:]
            elif location == 0 and location2 == -1:
                lines[index] = ""
                del_multiple_lines = True
            elif location > 0 and location2 > location:
                lines[index] = line[:location-1] + line[location2+2:]
            else:
                lines[index] = line[:location-1]
                del_multiple_lines = True
        location = line.find(COMMENT_1_LINE)
        if location > -1:
            if location == 0:
                lines[index] = ""
            else:
                lines[index] = line[:location-1]

    return lines

def delete_space(lines):
    for index, line in enumerate(lines):
        line = line.strip()
        line = re.sub(r"\s+(\(|\)|\<|\>|::|->|\.|,)", r"\1", line)
        line = re.sub(r"(\(|\)|\<|\>|::|->|\.|,)\s+", r"\1", line)
        if re.compile(r"^(::|->|\.)").search(line):
            i = index - 1
            while (i >= 0):
                if lines[i] == "":
                    i -= 1
                    continue
                line = lines[i] + line
                lines[i] = ""
                break
        if re.compile(r"(::|->|\.)$").search(line):
            line += lines[index + 1]
            lines[index + 1] = line
            line = ""
        lines[index] = line

    return lines

def get_name(lines):
    init_func_str = get_name_from_init(FUNC_ROS_INIT, lines)
    if init_func_str is "":
        init_func_str = get_name_from_init(FUNC_INIT, lines)

    args = init_func_str.split(COMMA)
    if len(args) != 3:
        # invalid init function
        return ""

    arg = get_value(args[2], lines)

    # delete single and double quotation
    return re.sub(r"\"|\''", "", arg)

def get_name_from_init(func_name, lines):
    get_init_func = False
    init_func_str = ""
    left_bracket_count = 0
    right_bracket_count = 0
    found = False

    for index,line in enumerate(lines):
        if get_init_func is True:
            for char in line:
                init_func_str += char
                if char is LEFT_BRACKET:
                    left_bracket_count += 1
                elif char is RIGHT_BRACKET:
                    right_bracket_count += 1
                    if left_bracket_count == right_bracket_count:
                        get_init_func = False
                        found = True
                        break
        else:
            location = line.find(func_name)
            if location > -1:
                line = line[location+len(func_name):]
                for char in line:
                    if left_bracket_count > 0:
                        init_func_str += char
                    if char is LEFT_BRACKET:
                        left_bracket_count += 1
                    elif char is RIGHT_BRACKET:
                        right_bracket_count += 1
                        if left_bracket_count == right_bracket_count:
                            get_init_func = False
                            found = True
                            break
                    get_init_func = True
        if found:
            # guard from getting multiple names
            break

    init_func_str = init_func_str[:-1]
    return init_func_str

def get_cluster_number(cluster_number, cluster_from, cluster_to):
    if cluster_from == 0 and cluster_to == 0:
        return 0

    if cluster_from == cluster_to:
        cluster_number = cluster_from
    elif cluster_number >= cluster_to:
        cluster_number = cluster_from
    elif cluster_number < cluster_from:
        cluster_number = cluster_from
    else:
        cluster_number += 1

    return cluster_number

def get_cpu_number(cpu_number, cpu_from, cpu_to):
    if cpu_from is None and cpu_to is None:
        return None

    if cpu_from == cpu_to:
        cpu_number = cpu_from
    elif cpu_number >= cpu_to:
        cpu_number = cpu_from
    elif cpu_number < cpu_from:
        cpu_number = cpu_from
    else:
        cpu_number += 1

    return cpu_number

def get_publish(lines):
    get_advertise_func = False
    advertise_func_str = ""
    left_bracket_count = 0
    right_bracket_count = 0
    args_list = []

    for index,line in enumerate(lines):
        if get_advertise_func is True:
            for char in line:
                advertise_func_str += char
                if char is LEFT_BRACKET:
                    left_bracket_count += 1
                elif char is RIGHT_BRACKET:
                    right_bracket_count += 1
                    if left_bracket_count == right_bracket_count:
                        get_advertise_func = False
                        advertise_func_str = advertise_func_str[:-1]
                        args = advertise_func_str.split(COMMA)
                        if len(args) < 2:
                            break
                        arg = get_value(args[0], lines)
                        args_list.append(arg)
        else:
            location = line.find(FUNC_ADVERTISE_1)
            if location == -1:
                location = line.find(FUNC_ADVERTISE_2)
            if location > -1:
                advertise_func_str = ""
                line = line[location+len(FUNC_ADVERTISE_1):]
                left_bracket_count = 0
                right_bracket_count = 0
                get_advertise_func = True
                for char in line:
                    if left_bracket_count > 0:
                        advertise_func_str += char
                    if char is LEFT_BRACKET:
                        left_bracket_count += 1
                    elif char is RIGHT_BRACKET:
                        right_bracket_count += 1
                        if left_bracket_count == right_bracket_count:
                            get_advertise_func = False
                            advertise_func_str = advertise_func_str[:-1]
                            args = advertise_func_str.split(COMMA)
                            if len(args) < 2:
                                break
                            arg = get_value(args[0], lines)
                            args_list.append(arg)

    return args_list


def get_publish_type(lines):
    get_advertise_func = False
    advertise_func_str = ""
    left_bracket_count = 0
    right_bracket_count = 0
    type_list = []

    for index,line in enumerate(lines):
        if get_advertise_func is True:
            for char in line:
                advertise_func_str += char
                if char is TEMPLATE_LEFT_BRACKET:
                    left_bracket_count += 1
                elif char is TEMPLATE_RIGHT_BRACKET:
                    right_bracket_count += 1
                    if left_bracket_count == right_bracket_count:
                        get_advertise_func = False
                        advertise_func_str = advertise_func_str[:-1]
                        type_list.append(advertise_func_str)
                elif char is LEFT_BRACKET and left_bracket_count == 0:
                    get_advertise_func = False
                    break
        else:
            location = line.find(FUNC_ADVERTISE_1)
            if location == -1:
                location = line.find(FUNC_ADVERTISE_2)
            if location > -1:
                advertise_func_str = ""
                line = line[location+len(FUNC_ADVERTISE_1):]
                left_bracket_count = 0
                right_bracket_count = 0
                get_advertise_func = True
                for char in line:
                    if left_bracket_count > 0:
                        advertise_func_str += char
                    if char is TEMPLATE_LEFT_BRACKET:
                        left_bracket_count += 1
                    elif char is TEMPLATE_RIGHT_BRACKET:
                        right_bracket_count += 1
                        if left_bracket_count == right_bracket_count:
                            get_advertise_func = False
                            advertise_func_str = advertise_func_str[:-1]
                            type_list.append(advertise_func_str)
                    elif char is LEFT_BRACKET and left_bracket_count == 0:
                        get_advertise_func = False
                        break
            else:
                continue

    return type_list

def get_subscribe(lines):
    get_subscribe_func = False
    subscribe_func_str = ""
    left_bracket_count = 0
    right_bracket_count = 0
    args_list = []

    for index,line in enumerate(lines):
        if get_subscribe_func is True:
            for char in line:
                subscribe_func_str += char
                if char is LEFT_BRACKET:
                    left_bracket_count += 1
                elif char is RIGHT_BRACKET:
                    right_bracket_count += 1
                    if left_bracket_count == right_bracket_count:
                        get_subscribe_func = False
                        subscribe_func_str = subscribe_func_str[:-1]
                        args = subscribe_func_str.split(COMMA)
                        if len(args) not in [3, 4]:
                            break
                        arg = get_value(args[0], lines)
                        args_list.append(arg)
        else:
            location = line.find(FUNC_SUBSCRIBE_1)
            if location == -1:
                location = line.find(FUNC_SUBSCRIBE_2)
            if location > -1:
                subscribe_func_str = ""
                line = line[location:]
                left_bracket_count = 0
                right_bracket_count = 0
                get_subscribe_func = True
                for char in line:
                    if left_bracket_count > 0:
                        subscribe_func_str += char
                    if char is LEFT_BRACKET:
                        left_bracket_count += 1
                    elif char is RIGHT_BRACKET:
                        right_bracket_count += 1
                        if left_bracket_count == right_bracket_count:
                            get_subscribe_func = False
                            subscribe_func_str = subscribe_func_str[:-1]
                            args = subscribe_func_str.split(COMMA)
                            if len(args) not in [3, 4]:
                                break
                            arg = get_value(args[0], lines)
                            args_list.append(arg)

    return args_list

def get_subscribe_type(lines):
    get_subscribe_func = False
    subscribe_func_str = ""
    left_bracket_count = 0
    right_bracket_count = 0
    args_list = []

    for index,line in enumerate(lines):
        if get_subscribe_func is True:
            for char in line:
                subscribe_func_str += char
                if char is LEFT_BRACKET:
                    left_bracket_count += 1
                elif char is RIGHT_BRACKET:
                    right_bracket_count += 1
                    if left_bracket_count == right_bracket_count:
                        get_subscribe_func = False
                        subscribe_func_str = subscribe_func_str[:-1]
                        args = subscribe_func_str.split(COMMA)
                        if len(args) == 3:
                            args_list.append(args[2])
                        elif len(args) == 4:
                            args_list.append(re.sub(r".*::", "", args[2]))
                        else:
                            break
        else:
            location = line.find(FUNC_SUBSCRIBE_1)
            if location == -1:
                location = line.find(FUNC_SUBSCRIBE_2)
            if location > -1:
                subscribe_func_str = ""
                line = line[location:]
                left_bracket_count = 0
                right_bracket_count = 0
                get_subscribe_func = True
                for char in line:
                    if left_bracket_count > 0:
                        subscribe_func_str += char
                    if char is LEFT_BRACKET:
                        left_bracket_count += 1
                    elif char is RIGHT_BRACKET:
                        right_bracket_count += 1
                        if left_bracket_count == right_bracket_count:
                            get_subscribe_func = False
                            subscribe_func_str = subscribe_func_str[:-1]
                            args = subscribe_func_str.split(COMMA)
                            if len(args) == 3:
                                args_list.append(args[2])
                            elif len(args) == 4:
                                args_list.append(re.sub(r".*::", "", args[2]))
                            else:
                                break
    subscribe_type_str = ""
    get_subscribe_type = False
    type_list = []
    for arg in args_list:
        for index,line in enumerate(lines):
            if get_subscribe_type is True:
                for char in line:
                    subscribe_type_str += char
                    if char is LEFT_BRACKET:
                        left_bracket_count += 1
                    elif char is RIGHT_BRACKET:
                        right_bracket_count += 1
                        if left_bracket_count == right_bracket_count:
                            get_subscribe_type = False
                            subscribe_type_str = subscribe_type_str[:-1]
                            m = re.search(PATTERN_SUBSCRIBE_1, subscribe_type_str)
                            if m is not None:
                                type_list.append(m.group().replace("::ConstPtr",""))
                                break
                            else:
                                m = re.search(PATTERN_SUBSCRIBE_2, subscribe_type_str)
                                if m is not None:
                                    type_list.append(m.group().replace("MessageEvent<","").replace(" const>",""))
                                    break
                                else:
                                    m = re.search(PATTERN_SUBSCRIBE_3, subscribe_type_str)
                                    if m is not None:
                                        type_list.append(m.group().replace("const ","").replace("ConstPtr&",""))
                                        break
                                    else:
                                        m = re.search(PATTERN_SUBSCRIBE_4, subscribe_type_str)
                                        if m is not None:
                                            type_list.append(m.group().replace("ConstPtr",""))
                                            break
                                        else:
                                            m = re.search(PATTERN_SUBSCRIBE_5, subscribe_type_str)
                                            if m is not None:
                                                type_list.append(m.group().replace("const ","").replace("&",""))
                                                break
            else:
                m = re.search(arg + r"[\s]*\(", line)
                if m is not None:
                    subscribe_type_str = ""
                    line = line[m.start():]
                    left_bracket_count = 0
                    right_bracket_count = 0
                    get_subscribe_type = True
                    for char in line:
                        if left_bracket_count > 0:
                            subscribe_type_str += char
                        if char is LEFT_BRACKET:
                            left_bracket_count += 1
                        elif char is RIGHT_BRACKET:
                            right_bracket_count += 1
                            if left_bracket_count == right_bracket_count:
                                get_subscribe_type = False
                                subscribe_type_str = subscribe_type_str[:-1]
                                m = re.search(PATTERN_SUBSCRIBE_1, subscribe_type_str)
                                if m is not None:
                                    type_list.append(m.group().replace("::ConstPtr",""))
                                    break
                                else:
                                    m = re.search(PATTERN_SUBSCRIBE_2, subscribe_type_str)
                                    if m is not None:
                                        type_list.append(m.group().replace("MessageEvent<","").replace(" const>",""))
                                        break
                                    else:
                                        m = re.search(PATTERN_SUBSCRIBE_3, subscribe_type_str)
                                        if m is not None:
                                            type_list.append(m.group().replace("const ","").replace("ConstPtr&",""))
                                            break
                                        else:
                                            m = re.search(PATTERN_SUBSCRIBE_4, subscribe_type_str)
                                            if m is not None:
                                                type_list.append(m.group().replace("ConstPtr",""))
                                                break
                                            else:
                                                m = re.search(PATTERN_SUBSCRIBE_5, subscribe_type_str)
                                                if m is not None:
                                                    type_list.append(m.group().replace("const ","").replace("&",""))
                                                    break

    return type_list


# start from here
if __name__ == '__main__':
    parser = ArgumentParser(usage=usage)
    parser.add_argument('dir_list', type=str, help='node directory list')
    parser.add_argument('-o', dest='outfile', type=str, help='output file')
    parser.add_argument('-c', dest='cluster_range', default='', type=str, help='cluster number range')
    parser.add_argument('--cpu', dest='cpu_range', default='', type=str, help='cpu number range')

    args = parser.parse_args()

    dir_list = args.dir_list.split(COMMA)
    map_list = []

    if args.cluster_range == '':
        cluster_from = 0
        cluster_to = 0
    else:
        cluster_range = args.cluster_range.split(HYPHEN)
        cluster_from = 9
        cluster_to = 16
        if (len(cluster_range) == 2):
            if (cluster_range[0] <= cluster_range[1]):
                cluster_from = int(cluster_range[0])
                cluster_to = int(cluster_range[1])

    if args.cpu_range == '':
        cpu_from = None
        cpu_to = None
    else:
        cpu_range = args.cpu_range.split(HYPHEN)
        cpu_from = 0
        cpu_to = 4
        if (len(cpu_range) == 2):
            if (cpu_range[0] <= cpu_range[1]):
                cpu_from = int(cpu_range[0])
                cpu_to = int(cpu_range[1])
                cluster_from = 1
                cluster_to = sys.maxint

    cluster_number = 0
    cpu_number = -1

    for dir_name in dir_list:
        file_list = glob.glob(dir_name + "/*.cpp")
        # read all lines in all c++ files
        lines = []
        for cppfile in file_list:
            with open(cppfile, "r") as test_data:
                lines.extend(test_data.readlines())

        lines = delete_comment(lines)
        lines = delete_space(lines)

        cluster_number = get_cluster_number(cluster_number, cluster_from, cluster_to)
        cpu_number = get_cpu_number(cpu_number, cpu_from, cpu_to)

        name = get_name(lines)
        if name == "":
            print("[Error] Failed to generate map : Invalid definition of initialize function")
            exit()
        publish_list = get_publish(lines)
        publish_type_list = get_publish_type(lines)
        if len(publish_list) != len(publish_type_list):
            print("[Error] Failed to generate map : Invalid definition of publish function")
            exit()
        subscribe_list = get_subscribe(lines)
        subscribe_type_list = get_subscribe_type(lines)
        if len(subscribe_list) != len(subscribe_type_list):
            print("[Error] Failed to generate map : Invalid definition of subscribe function")
            exit()

        map = MapInformation(
            name,
            cluster_number,
            cpu_number,
            publish_list,
            publish_type_list,
            subscribe_list,
            subscribe_type_list
        )

        map_list.append(map)

    out_file = args.outfile

    with open(out_file, "w") as stream:
        for map_data in map_list:
            map_data.write_map(stream)
