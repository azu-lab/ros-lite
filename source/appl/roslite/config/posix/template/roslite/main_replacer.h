{% include 'common/note.cpp' %}


#pragma once

// #ifdef NODENAME_MAIN
//     #define main(argc, argv) NODENAME_main(argc, argv)
// #endif

{% for node in node_list %}
#ifdef  {{ node['name'] }}_MAIN
    #define main(argc, argv) {{ node['name'] }}_main(argc, argv)
#endif
{% endfor %}

