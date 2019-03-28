// [note] Auto-generated file
// [note] 2019-03-28T05:27:45Z
// [note] based on source/appl/ros_src/map/roslite_map_two_listenres.map



#if ROSLITE_TARGET_CLUSTER_ID == 0
  #include "roslite/include/ros/init.h"
  #include "roslite/include/ros/thread.h"
  #include "roslite/include/ros/debug.h"
#else
  #include "ros/init.h"
  #include "ros/thread.h"
  #include "ros/debug.h"
#endif

extern void cluster0_main();

extern int talker_main(int argc, char **argv);
extern int listener_main(int argc, char **argv);
extern int listener2_main(int argc, char **argv);

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
    create_thread(1024 * 10, cluster0_main);
#elif ROSLITE_TARGET_CLUSTER_ID == 1
    ROSLITE_NAMESPACE::createAndStartNodeThread(talker_main);
#elif ROSLITE_TARGET_CLUSTER_ID == 2
    ROSLITE_NAMESPACE::createAndStartNodeThread(listener_main);
    ROSLITE_NAMESPACE::createAndStartNodeThread(listener2_main);
#endif 

    roslite_debug_printf("--------- cluster end ---------\n");
} 

