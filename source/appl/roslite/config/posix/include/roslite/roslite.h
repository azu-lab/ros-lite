#pragma once

#include <cstdint>

#ifdef __cplusplus
extern "C"
{
#endif

#ifndef ROSLITE_TARGET_CLUSTER_ID
#define ROSLITE_TARGET_CLUSTER_ID 0
#endif

#define ROSLITE_LCID_ANY -1

#define ROSLITE_NSID_DEFAULT 0
#define ROSLITE_NSID_GLOBAL 1

#define ROSLITE_MSG_SEND_ASYNC 1
#define ROSLITE_MSG_RECV_SYNC 0

#define ROSLITE_EOK 0
#define ROSLITE_EFAIL -1

typedef int roslite_id_t;
typedef int roslite_erid_t;
typedef int roslite_er_t;
typedef int roslite_generic_id_t;
typedef int roslite_message_fd_t;

struct _roslite_thread_info {
    roslite_id_t    lcid;
};
typedef struct _roslite_thread_info roslite_thread_info_t;

struct _roslite_thread_attr {};
typedef struct _roslite_thread_attr roslite_thread_attr_t;

extern roslite_er_t roslite_thread_attr_init(roslite_thread_attr_t *attr);
extern roslite_erid_t roslite_thread_create(const roslite_thread_attr_t *attr, void (*entry)(uint32_t stacd, uintptr_t exinf), uintptr_t exinf);
extern roslite_er_t roslite_thread_start(roslite_id_t tid, uint32_t start_code);
extern roslite_id_t roslite_thread_getid();
extern roslite_er_t roslite_thread_get_info(roslite_id_t tid, roslite_thread_info_t *thread_info);
extern void roslite_thread_delay(uint32_t time_ms);

extern roslite_er_t roslite_name_register(const char *name, roslite_generic_id_t id, roslite_id_t nsid);
extern roslite_generic_id_t roslite_name_lookup(const char *name, roslite_id_t nsid);

extern roslite_message_fd_t roslite_message_connect(roslite_id_t tid);
extern roslite_message_fd_t roslite_message_accept();
extern roslite_er_t roslite_message_close(roslite_message_fd_t fd);

extern roslite_er_t roslite_message_send(roslite_message_fd_t fd, const void *data, uint32_t size, uint32_t flags);
extern roslite_er_t roslite_message_receive(roslite_message_fd_t fd, void *data, uint32_t *size, uint32_t flags);

#define ROSLITE_BREAK_ERROR() \
    roslite_thread_info_t thread_ref; \
    roslite_thread_get_info(roslite_thread_getid(), &thread_ref); \
    roslite_debug_printf("[ERROR]: hwclid=%d, lcid=%d, tid=%d\n", ROSLITE_TARGET_CLUSTER_ID, thread_ref.lcid, roslite_thread_getid() ); \
    roslite_debug_break();

extern int roslite_debug_printf(const char* format, ...);
extern void roslite_debug_break();

#ifdef __cplusplus
}
#endif
