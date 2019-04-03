#!/usr/bin/env python

import sys
import os
import stat
import yaml
from collections import defaultdict
from datetime import datetime
import itertools

import colorful
from jinja2 import Environment, FileSystemLoader

import commons  # local module


def get_topics(yaml_data):
    s = set()
    for node in yaml_data:
        for topic in node['publish']:
            s.add(topic)
        for topic in node['subscribe']:
            s.add(topic)
    return sorted(s)


def get_clusters(yaml_data):
    s = set()
    for node in yaml_data:
        s.add(node['cluster'])
    return sorted(s)


def get_cpus(yaml_data):
    s = defaultdict(set)
    for node in yaml_data:
        if 'cpu' in node:
            s[node['cluster']].add(node['cpu'])
    return {k: sorted(v) for k, v in s.items()}


def get_subscriber_infos(yaml_data, cluster):
    subscriber_info_list = []

    counts = defaultdict(int)
    for node in yaml_data:
        if node['cluster'] == cluster:
            continue
        for topic in node['subscribe']:
            counts_key = "%s#%d" % (topic, node['cluster'])
            name = '_'.join((topic.replace('/', '_'), str(node['cluster']), str(counts[counts_key])))
            subscriber_info_list.append({
                "topic": topic,
                "name": name,
                "cluster": node['cluster']
            })
            counts[counts_key] += 1

    if cluster != 0:
        all_pub_topics = set()
        for node in yaml_data:
            for topic in node['publish']:
                all_pub_topics.add(topic)
        for topic in all_pub_topics:
            name = '_'.join((topic.replace('/', '_'), '0', '0'))
            subscriber_info_list.append({
                "topic": topic,
                "name": name,
                "cluster": 0
            })

    return subscriber_info_list


def get_topic_infos_impl(yaml_data, topic_key, type_key):
    topic_info_map = {}
    
    for node in yaml_data:
        for topic, topic_type in zip(node[topic_key], node[type_key]):
            type_package = ''
            type_name = ''
            type_elems = topic_type.split('::')
            if len(type_elems) == 1:
                type_package = ''
                type_name = type_elems[0]
            else:
                type_package = type_elems[0]
                type_name = type_elems[-1]
                topic_info_map[topic] = {
                    'type_package' : type_package, # e.g. std_msgs
                    'type_name' : type_name,       # e.g. String
                    'topic' : topic,               # e.g. /chatter
                    'topic_no_slash' : topic.replace('/', '_'), # e.g. _chatter
                    'type' : topic_type            # e.g. std_msgs::String
                }

    return topic_info_map.values()

    
def get_topic_infos(yaml_data):
    return get_topic_infos_impl(yaml_data, 'publish', 'publish_type'), get_topic_infos_impl(yaml_data, 'subscribe', 'subscribe_type')


def generate(yaml_file):
    with open('%s' % (yaml_file)) as fp:
        yaml_data = yaml.safe_load(fp)

    topic_list = get_topics(yaml_data)
    print('')
    print('All topics: {}'.format(colorful.yellow(topic_list)))
    cluster_list = get_clusters(yaml_data)
    print('All activated clusters: {}'.format(colorful.yellow(cluster_list)))
    cluster_cpu_map = get_cpus(yaml_data)
    print('All cluster-cpu mappings: {}'.format(colorful.yellow(cluster_cpu_map)))
    print('')
    subscriber_info_list_map = {
        cluster: get_subscriber_infos(yaml_data, cluster)
        for cluster
        in itertools.chain([0], cluster_list)
    }
    pub_topic_infos, sub_topic_infos = get_topic_infos(yaml_data)
    type_package_list = list(set([topic_info['type_package'] for topic_info in itertools.chain(pub_topic_infos, sub_topic_infos) if len(topic_info['type_package']) > 0]))

    template_path = os.path.join(os.path.dirname(os.path.abspath(__file__)), os.pardir, 'config', 'posix', 'template')
    env = Environment(loader=FileSystemLoader(template_path), trim_blocks=True, lstrip_blocks=True)

    env.globals['datetime'] = datetime
    env.globals['input_file'] = os.path.relpath(yaml_file)

    output_dir = os.path.join(os.path.dirname(os.path.abspath(__file__)), os.pardir, os.pardir)

    filepath = 'roslite/src/generated/init.cpp'
    with open(os.path.join(output_dir, filepath), 'w') as f:
        f.write(
            env.get_template('roslite/init.cpp').render(
                topic_list=topic_list,
                cluster_list=itertools.chain([0], cluster_list),
                subscriber_info_list_map=subscriber_info_list_map
            )
        )
    commons.print_generated_file(filepath)

    filepath = 'ros_src/generated/init_threads.cpp'
    with open(os.path.join(output_dir, filepath), 'w') as f:
        f.write(
            env.get_template('ros_src/init_threads.cpp').render(
                node_list=yaml_data,
                cluster_list=cluster_list
            )
        )
    commons.print_generated_file(filepath)

    filepath = 'ros_src/generated/roslite_node.cmake'
    with open(os.path.join(output_dir, filepath), 'w') as f:
        f.write(
            env.get_template('ros_src/roslite_node.cmake').render(
                node_list=yaml_data,
                cluster_list=cluster_list
            )
        )
    commons.print_generated_file(filepath)

    filepath = 'ros_src/generated/roslite_app.cmake'
    with open(os.path.join(output_dir, filepath), 'w') as f:
        f.write(
            env.get_template('ros_src/roslite_app.cmake').render(
                node_list=yaml_data,
                cluster_list=cluster_list
            )
        )
    commons.print_generated_file(filepath)

    filepath = 'roslite/generated/roslite.cmake'
    with open(os.path.join(output_dir, filepath), 'w') as f:
        f.write(
            env.get_template('roslite/roslite.cmake').render(
                node_list=yaml_data,
                cluster_list=cluster_list
            )
        )
    commons.print_generated_file(filepath)

    filepath = 'roslite/include/ros/generated/main_replacer.h'
    with open(os.path.join(output_dir, filepath), 'w') as f:
        f.write(
            env.get_template('roslite/main_replacer.h').render(
                node_list=yaml_data,
                cluster_list=cluster_list
            )
        )
    commons.print_generated_file(filepath)

    filepath = 'roslite/scripts/roslite_cli/generated/rosl_run'
    with open(os.path.join(output_dir, filepath), 'w') as f:
        f.write(
            env.get_template('roslite/rosl_run').render(
                node_list=yaml_data,
                cluster_list=cluster_list,
                cluster_cpu_map=cluster_cpu_map
            )
        )
    current_umask = os.umask(0o0)
    os.umask(current_umask)
    os.chmod(os.path.join(output_dir, filepath), (stat.S_IRWXU | stat.S_IRWXG | stat.S_IRWXO) & ~current_umask)
    commons.print_generated_file(filepath)

    filepath = 'ros_bridge/generated/ros_bridge_generated.cmake'
    with open(os.path.join(output_dir, filepath), 'w') as f:
        f.write(
            env.get_template('ros_bridge/ros_bridge_generated.cmake').render(
                type_package_list=type_package_list
            )
        )
    commons.print_generated_file(filepath)

    filepath = 'ros_bridge/generated/ros_bridge_generated.cpp'
    with open(os.path.join(output_dir, filepath), 'w') as f:
        f.write(
            env.get_template('ros_bridge/ros_bridge_generated.cpp').render(
                pub_topic_infos=pub_topic_infos,
                sub_topic_infos=sub_topic_infos
            )
        )
    commons.print_generated_file(filepath)


# start from here
if __name__ == '__main__':
    generate(sys.argv[1])
