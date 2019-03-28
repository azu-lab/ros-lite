{% include 'common/note.cmake' %}


{% for package in type_package_list %}
find_package({{package}} REQUIRED)
list(APPEND ros_bridge_INCLUDE_DIRS {{ '${' }}{{package}}{{'_INCLUDE_DIRS}' }})
list(APPEND ros_bridge_LIBRARIES {{ '${' }}{{package}}{{'_LIBRARIES}' }})
{% endfor %}
