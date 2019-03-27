{% include 'common/note.cmake' %}


find_package(roscpp)
list(APPEND ros_LIBRARIES ${roscpp_LIBRARIES})

{% for package in type_package_list %}
find_package({{package}})
list(APPEND ros_INCLUDE_DIRS {{ '${' }}{{package}}{{'_INCLUDE_DIRS}' }})
list(APPEND ros_LIBRARIES {{ '${' }}{{package}}{{'_LIBRARIES}' }})
{% endfor %}
