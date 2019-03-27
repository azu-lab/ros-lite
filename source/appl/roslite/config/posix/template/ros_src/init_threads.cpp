{% include 'common/note.cpp' %}


{% include 'common/generic_includes.cpp' %}


#if ROSLITE_TARGET_CLUSTER_ID == 0
  #include "roslite/include/ros/init.h"
  #include "roslite/include/ros/thread.h"
  #include "roslite/include/ros/debug.h"
#else
  #include "ros/init.h"
  #include "ros/thread.h"
  #include "ros/debug.h"
#endif

extern void ros_bridge_main();

{% for node in node_list %}
extern int {{node['name']}}_main(int argc, char **argv);
{% endfor %}

static void
thread_func(uint32_t stacd, uintptr_t exinf)
{
    void (*func)() = (void (*)())exinf;
    func();
}

static void
create_thread(uint32_t stack_size, void (*func)())
{
    roslite_thread_attr_t attr;
    roslite_thread_attr_init(&attr);
    roslite_erid_t tid = roslite_thread_create(&attr, thread_func, (uintptr_t)func);
    ROSLITE_CHECK_SIMPLE(tid);

    roslite_er_t ret = roslite_thread_start((roslite_id_t)tid, 0);
    ROSLITE_CHECK_SIMPLE(ret);
}

extern "C" void 
create_init_threads(){ 

#if ROSLITE_TARGET_CLUSTER_ID == 0
#else
    /* Initialize servers in clusters */
    // roslite_rpc_client_init();
    // roslite_async_init();

    /* Synchronize all PE0 of all booted cluster */
    // roslite_rpc_barrier_all();
#endif

    roslite_debug_printf("--------- cluster start: cluster_id=%d ---------\n", ROSLITE_TARGET_CLUSTER_ID);

    ROSLITE_NAMESPACE::generated_init();

#if ROSLITE_TARGET_CLUSTER_ID == 0
    create_thread(1024 * 10, ros_bridge_main);
{% for cluster in cluster_list: %}
#elif ROSLITE_TARGET_CLUSTER_ID == {{cluster}}
    {% for node in node_list if node['cluster'] == cluster %}
    ROSLITE_NAMESPACE::createAndStartNodeThread({{node['name']}}_main);
    {% endfor %}
{% endfor %}
#endif 

#if ROSLITE_TARGET_CLUSTER_ID != 17
    // TODO
    // roslite_rpc_barrier_all();  /* Synchronize all PE0 of all booted cluster */
    roslite_debug_printf("--------- cluster end ---------\n");
#endif 
} 


