{% include 'common/note.cmake' %}


#/****************************************************************************
 #[ roslite_node.cmake ] - cmake rules for roslite nodes
#****************************************************************************/

{% for cluster in cluster_list: %}

{% for node in node_list if node['cluster'] == cluster %}
aux_source_directory({{ '${CMAKE_CURRENT_SOURCE_DIR}/nodes/' }}{{ node['name'] }} {{ node['name'] }}_SOURCE)
add_library(
        ros_node-{{ cluster }}-{{ node['name'] }} OBJECT
        {{ '${' }}{{ node['name'] }}{{ '_SOURCE}' }}
)
target_compile_definitions(
        ros_node-{{ cluster }}-{{ node['name'] }}
        PUBLIC ROSLITE_TARGET_CLUSTER_ID={{ cluster }}
        PUBLIC {{ node['name'] }}_MAIN=1
)
target_include_directories(
        ros_node-{{ cluster }}-{{ node['name'] }}
        PRIVATE {{ '${CMAKE_CURRENT_SOURCE_DIR}/nodes/' }}{{ node['name'] }}
)
{% endfor %}

add_library(
        ros_node-{{ cluster }} STATIC
{% for node in node_list if node['cluster'] == cluster %}
        $<TARGET_OBJECTS:ros_node-{{ cluster }}-{{ node['name'] }}>
{% endfor %}
)

target_compile_definitions(
        ros_node-{{ cluster }}
        PUBLIC ROSLITE_TARGET_CLUSTER_ID={{ cluster }}
)
{% endfor %}
