#if ROSLITE_TARGET_CLUSTER_ID == 0
  #include "roslite/include/ros/thread.h"
  #include "roslite/include/ros/debug.h"
#else
  #include "ros/thread.h"
  #include "ros/debug.h"
#endif

namespace ROSLITE_NAMESPACE
{
    void entryNodeThread(uint32_t start_code, uintptr_t exinf){
        int (*node_main)(int, char**) = (int(*)(int, char**)) exinf;
        node_main(0, NULL);
    }

    void createAndStartNodeThread(int (*node_main)(int, char**)){
        /* crete thread */
        roslite_thread_attr_t attr;
        roslite_id_t tid;
        roslite_er_t ret;
        roslite_thread_attr_init(&attr);
        tid = roslite_thread_create(&attr, entryNodeThread, (uintptr_t) node_main);
        ROSLITE_CHECK_SIMPLE(tid);

        ret = roslite_thread_start(tid, 0);
        ROSLITE_CHECK_SIMPLE(ret);
    }
    
}  // namespace ROSLITE_NAMESPACE
