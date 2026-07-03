#pragma once

#include <Eigen/Dense>

#ifdef LOG_PATH
#define LOG LOG_PATH "logs.log"
#else
#define LOG NULL
#endif

using scalar_t = double;
using vector_t = Eigen::Matrix<scalar_t, Eigen::Dynamic, 1>;
using matrix_t = Eigen::Matrix<scalar_t, Eigen::Dynamic, Eigen::Dynamic>;
using vector3_t = Eigen::Matrix<scalar_t, 3, 1>;
using vector1_t = Eigen::Matrix<scalar_t, 1, 1>;
using matrix3_t = Eigen::Matrix<scalar_t, 3, 3>;
using quaternion_t = Eigen::Quaternion<scalar_t>;

class ThreadLock
{
public:
    ThreadLock() = default;
    ~ThreadLock() = default;
    void Lock()   { while(this->lock.exchange(true, std::memory_order_acquire));}
    void Unlock() { this->lock.store(false, std::memory_order_release);}    
private:
    std::atomic_bool lock{false};

};

#include <stdio.h>
#include <time.h>
#include <sys/file.h>

// 简单的日志写入函数
inline void write_log(const char* message, const char* level="", const char* filename=LOG) {
    FILE* logfile = fopen(filename, "a");
    if (logfile == NULL) {
        fprintf(stderr, "Failed to open log file: %s\n", filename);
        return;
    }
    
    // 获取文件锁（多进程安全）
    int fd = fileno(logfile);
    flock(fd, LOCK_EX);
    
    // 获取时间（使用线程安全的版本）
    time_t now = time(NULL);
    struct tm tm_buf;
    struct tm* t = localtime_r(&now, &tm_buf);  // 线程安全
    
    // 写入日志
    fprintf(logfile, "[%04d-%02d-%02d %02d:%02d:%02d]: [%s] %s\n",
            t->tm_year + 1900, t->tm_mon + 1, t->tm_mday,
            t->tm_hour, t->tm_min, t->tm_sec,
            level, message);
    
    fflush(logfile);  // 确保写入磁盘
    
    // 释放锁并关闭
    flock(fd, LOCK_UN);
    fclose(logfile);
}

struct Proprioception{
    struct obsScale{
        scalar_t baseAngVel;
        scalar_t projectedGravity;
        scalar_t jointPos;
        scalar_t jointVel;
    };
    vector1_t sin;
    vector1_t cos;
    vector3_t baseAngVel;
    vector3_t projectedGravity;
    vector_t jointPos;
    vector_t jointVel;

    scalar_t clipObs;
    obsScale scale;
};


struct CommandRange {
    double x_min, x_max;
    double y_min, y_max;
    double yaw_min, yaw_max;
};

struct Command{
    vector3_t base_vel;
};


struct Actions{
    scalar_t clipActions;
    vector_t actionScale;
};


struct JointState{
    vector_t defaultJointPos;
    vector_t effort_limit;
    vector_t velocity_limit;
};

