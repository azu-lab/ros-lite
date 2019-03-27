#include "roslite/roslite.h"

#include <vector>
#include <map>
#include <string>
#include <fstream>

#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <cstdarg>

#include <pthread.h>

#include <unistd.h>
#include <signal.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <sys/un.h>
#include <sys/syscall.h>

#ifdef __cplusplus
extern "C"
{
#endif

extern void create_init_threads(void);

#ifdef __cplusplus
}
#endif

static std::string _roslite_shm_prefix("/dev/shm/roslite--");

static roslite_message_fd_t _create_message_sock(roslite_id_t tid);

struct roslite_thread {
    pid_t tid = -1;
    pthread_t pthread;

    void (*entry)(uint32_t stacd, uintptr_t exinf);
    uintptr_t exinf;
    uint32_t start_code;
    
    enum status_t {CREATED, INITIALIZED, STARTED};
    status_t status = CREATED;

    pthread_cond_t status_cond = PTHREAD_COND_INITIALIZER;
    pthread_mutex_t status_mutex = PTHREAD_MUTEX_INITIALIZER;

    std::string sock_path;
    int sock_fd;

    roslite_thread_info_t info;
};

static std::map<roslite_id_t, roslite_thread *> _roslite_thread_map;

roslite_er_t roslite_thread_attr_init(roslite_thread_attr_t *attr) {
    // stub
    return 0;
}

static void * _roslite_pthread_main(void *arg) {
    roslite_thread *thread = (roslite_thread *)arg;

    roslite_id_t tid = roslite_thread_getid();
    thread->tid = tid;
    thread->sock_fd = _create_message_sock(tid);

    pthread_mutex_lock(&thread->status_mutex);
    thread->status = roslite_thread::INITIALIZED;
    pthread_cond_signal(&thread->status_cond);
    pthread_mutex_unlock(&thread->status_mutex);

    pthread_mutex_lock(&thread->status_mutex);
    do {
        pthread_cond_wait(&thread->status_cond, &thread->status_mutex);
    } while (thread->status != roslite_thread::STARTED);
    pthread_mutex_unlock(&thread->status_mutex);

    thread->entry(thread->start_code, thread->exinf);

    return NULL;
}

roslite_erid_t roslite_thread_create(const roslite_thread_attr_t *attr, void (*entry)(uint32_t stacd, uintptr_t exinf),
                                     uintptr_t exinf) {
    roslite_debug_printf("roslite_thread_create attr:%p, entry:%p, exinf:%p\n", attr, entry, exinf);

    roslite_thread *thread = new roslite_thread();
    thread->entry = entry;
    thread->exinf = exinf;

    pthread_create(&thread->pthread, NULL, _roslite_pthread_main, thread);

    pthread_mutex_lock(&thread->status_mutex);
    do {
        pthread_cond_wait(&thread->status_cond, &thread->status_mutex);
    } while (thread->status != roslite_thread::INITIALIZED);
    pthread_mutex_unlock(&thread->status_mutex);

    _roslite_thread_map[thread->tid] = thread;

    return thread->tid;
}

roslite_er_t roslite_thread_start(roslite_id_t tid, uint32_t start_code) {
    roslite_thread *thread = _roslite_thread_map.at(tid);

    roslite_debug_printf("roslite_thread_start tid:%d, start_code:%d\n", tid, start_code);

    pthread_mutex_lock(&thread->status_mutex);
    thread->start_code = start_code;
    thread->status = roslite_thread::STARTED;
    pthread_cond_signal(&thread->status_cond);
    pthread_mutex_unlock(&thread->status_mutex);

    return ROSLITE_EOK;
}

roslite_id_t roslite_thread_getid() {
    return (roslite_id_t)syscall(SYS_gettid);
}

roslite_er_t roslite_thread_get_info(roslite_id_t tid, roslite_thread_info_t *thread_info) {
    roslite_thread *thread = _roslite_thread_map.at(tid);
    *thread_info = thread->info;

    return ROSLITE_EOK;
}

void roslite_thread_delay(uint32_t time_ms)
{
    usleep(time_ms * 1000);
}

static std::string _sanitized_filename_string(std::string& str) {
    auto sanitized = str;
    return sanitized;
}

static std::string _roslite_tid_path(std::string name) {
    return _roslite_shm_prefix + _sanitized_filename_string(name) + ".tid";
}

static std::string _roslite_sock_path(roslite_id_t tid) {
    return _roslite_shm_prefix + std::to_string(tid) + ".sock";
}

static std::vector<std::string> _roslite_name_files;

roslite_er_t roslite_name_register(const char *name, roslite_generic_id_t id, roslite_id_t nsid) {
    roslite_debug_printf("roslite_name_register name:%s, id:%d, nsid:%d\n", name, id, nsid);

    std::string path(_roslite_tid_path(std::string(name)));

    _roslite_name_files.push_back(path);

    std::ofstream ofs(path);
    ofs << id;
    ofs.close();

    return ROSLITE_EOK;
}

roslite_generic_id_t roslite_name_lookup(const char *name, roslite_id_t nsid) {
    roslite_debug_printf("roslite_name_lookup name:%s, nsid:%d\n", name, nsid);

    roslite_generic_id_t id = -1;

    std::ifstream ifs(_roslite_tid_path(std::string(name)));
    if (ifs) {
        ifs >> id;
        ifs.close();
    }

    return id;
}

static roslite_message_fd_t _create_message_sock(roslite_id_t tid) {
    int sock = socket(AF_UNIX, SOCK_STREAM, 0);

    std::string sock_path = _roslite_sock_path(tid);

    struct sockaddr_un sa = {0};
    sa.sun_family = AF_UNIX;
    strncpy(sa.sun_path, sock_path.c_str(), sizeof(sa.sun_path) - 1);

    if (bind(sock, (struct sockaddr *)&sa, sizeof(struct sockaddr_un)) == -1){
        return ROSLITE_EFAIL;
    }

    if (listen(sock, 1024) == -1){
        return ROSLITE_EFAIL;
    }

    return sock;
}

static void _remove_message_sock(roslite_id_t tid) {
    std::string sock_path = _roslite_sock_path(tid);
    unlink(sock_path.c_str());
}

roslite_message_fd_t roslite_message_connect(roslite_id_t tid) {
    int sock = socket(AF_UNIX, SOCK_STREAM, 0);

    std::string sock_path = _roslite_sock_path(tid);

    struct sockaddr_un sa;
    memset(&sa, 0, sizeof(struct sockaddr_un));
    sa.sun_family = AF_UNIX;
    strncpy(sa.sun_path, sock_path.c_str(), sizeof(sa.sun_path) - 1);

    if (connect(sock, (struct sockaddr *)&sa, sizeof(struct sockaddr_un)) == -1){
        return ROSLITE_EFAIL;
    }

    return sock;
}

roslite_message_fd_t roslite_message_accept() {
    roslite_thread *thread = _roslite_thread_map[roslite_thread_getid()];

    roslite_debug_printf("roslite_message_accept fd:%d\n", thread->sock_fd);

    return accept(thread->sock_fd, NULL, NULL);
}

roslite_er_t roslite_message_close(roslite_message_fd_t fd) {
    close(fd);
    return ROSLITE_EOK;
}

roslite_er_t roslite_message_send(roslite_message_fd_t fd, const void *data, uint32_t size, uint32_t flags) {
    roslite_debug_printf("roslite_message_send fd:%d, data:%p, size:%d, flags:%d\n", fd, data, size, flags);

    int rval = send(fd, data, size, 0);
    if (rval <= 0) {
        return ROSLITE_EFAIL;
    }

    return ROSLITE_EOK;
}

roslite_er_t roslite_message_receive(roslite_message_fd_t fd, void *data, uint32_t *size, uint32_t flags) {
    roslite_debug_printf("roslite_message_receive fd:%d, data:%p, size:%d, flags:%d\n", fd, data, *size, flags);

    int received_size = 0;
    while (*size > received_size) {
        int rval = recv(fd, (char *)data + received_size, *size - received_size, 0);
        if (rval <= 0) {
            perror("failed to receive message");
            *size = received_size;
            return ROSLITE_EFAIL;
        }
        received_size += rval;
    }

    return ROSLITE_EOK;
}

static void _cleanup() {
    for (auto path : _roslite_name_files) {
        unlink(path.c_str());
    }

    for (auto entry : _roslite_thread_map) {
        _remove_message_sock(entry.second->tid);
    }
}

static void _term_handler(int signum) {
    exit(0);
}

int main(int argc, char *argv[]) {
    atexit(_cleanup);

    struct sigaction action;
    memset(&action, 0, sizeof(struct sigaction));
    action.sa_handler = _term_handler;
    sigaction(SIGINT, &action, NULL);
    sigaction(SIGTERM, &action, NULL);

    create_init_threads();

    for (auto entry : _roslite_thread_map) {
        void *rval = NULL;
        pthread_join(entry.second->pthread, &rval);
    }

    return ROSLITE_EOK;
}

int roslite_debug_printf(const char *format, ...) {
    va_list ap;
    va_start(ap, format);
    int result = vfprintf(stderr, format, ap);
    va_end(ap);
    return result;
}

void roslite_debug_break() {
    abort();
}
