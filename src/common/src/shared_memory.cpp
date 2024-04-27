//
// Created by plutoli on 2021/6/13.
//

#include <iostream>
#include <memory.h>
#include "shared_memory.h"

// 构造函数
SharedMemory::SharedMemory():
        isInitialized_(false),
        key_(0),
        size_(0),
        sharedMemoryID_(-1),
        semaphoreID_(-1),
        sharedMemoryAddress_(nullptr)
{
}

// 获取共享内存的初始化状态
bool SharedMemory::GetStatus() const
{
    return isInitialized_;
}

// 获取共享内存和数据同步信号量的键
key_t SharedMemory::GetKey() const
{
    return key_;
}

// 获取共享内存的字节长度
size_t SharedMemory::GetSize() const
{
    return size_;
}

// 初始化共享内存
bool SharedMemory::Init(const key_t &key, const size_t &size)
{
    // 排他性创建共享内存
    sharedMemoryID_ = shmget(key, size, ACCESS_BIT|IPC_CREAT|IPC_EXCL);
    if ((sharedMemoryID_ < 0) && (errno == EEXIST))
    {
        // 如果共享内存已经存在，则挂载共享内存；在挂载时，必须设置size=0
        spdlog::warn("SharedMemory has been created. key = {}", key);
        sharedMemoryID_ = shmget(key, 0, ACCESS_BIT|IPC_CREAT);
    }

    // 判断共享内存是否获取成功
    if (sharedMemoryID_ < 0)
    {
        spdlog::error("SharedMemory get failure. key = {}, errno = {}", key, errno);
        return false;
    }
    else
    {
        spdlog::info("SharedMemory get success. key = {}", key);
    }

    // 把共享内存连接到当前进程的地址空间
    if (sharedMemoryAddress_ == nullptr)
    {
        sharedMemoryAddress_ = static_cast<unsigned char *>(shmat(sharedMemoryID_, nullptr, 0));
    }

    // 判断共享内存是否连接成功
    if (sharedMemoryAddress_ == (unsigned char *)-1)
    {
        spdlog::error("SharedMemory attach failure.  key = {}, errno = {}", key, errno);
        return false;
    }
    else
    {
        spdlog::info("SharedMemory attach success.  key = {}", key);
    }

    // 排他性创建信号量
    semaphoreID_ = semget(key,1,ACCESS_BIT|IPC_CREAT|IPC_EXCL);
    if (semaphoreID_ >= 0)
    {
        // 如果创建成功，初始化信号量
        union semun sem_union;
        sem_union.val = 1;
        if(semctl(semaphoreID_, 0, SETVAL, sem_union) < 0)
        {
            spdlog::error("Semaphore set value failure, computer should be reboot. key = {}, errno = {}", key, errno);
            return false;
        }
        // TODO 该部分代码如果出现异常，可能会导致程序死锁。
        //      1、问题描述：如果在排他性创建信号量之后、新创建的信号量初始化成功之前出现异常，因为新创建的信号量
        //         没有赋予初值，将会导致后面的Semaphore_P()操作一直挂起，程序没有办法获取共享内存的读写权限。
        //      2、解决方案：可以将信号量的排他性创建和初始化做成原子操作。
    }
    else
    {
        if (errno == EEXIST)
        {
            // 如果信号量已经存在，则挂载信号量
            semaphoreID_ = semget(key,1,ACCESS_BIT|IPC_CREAT);
        }
    }

    // 判断信号量是否获取成功
    if (semaphoreID_ < 0)
    {
        spdlog::error("Semaphore get failure. key = {}, errno = {}", key, errno);
        return false;
    }
    else
    {
        spdlog::info("Semaphore get success. key = {}", key);
    }

    // 更新私有数据
    key_ = key;
    size_ = size;
    isInitialized_ = true;

    // 返回初始化结果
    return true;
}

// 清理共享内存资源
bool SharedMemory::Clear()
{
    // 判断共享内存是否已经初始化
    if (!isInitialized_)
    {
        return true;
    }

    // 将共享内存从当前进程中分离
    if(shmdt(sharedMemoryAddress_) < 0)
    {
        spdlog::error("SharedMemory datach failure. key = {}, errno = {}", key_, errno);
        return false;
    }
    else
    {
        spdlog::info("SharedMemory datach success. key = {}", key_);
    }

    // 删除共享内存
    if(shmctl(sharedMemoryID_, IPC_RMID, nullptr) < 0)
    {
        spdlog::error("SharedMemory delete failure. key = {}, errno = {}", key_, errno);
        return false;
    }
    else
    {
        spdlog::info("SharedMemory delete success. key = {}", key_);
    }

    // 删除信号量
    if (semctl(semaphoreID_, 0, IPC_RMID) < 0)
    {
        spdlog::error("Semaphore delete failure. key = {}, errno = {}", key_, errno);
        return false;
    }
    else
    {
        spdlog::info("Semaphore delete success. key = {}", key_);
    }

    // 重置私有数据
    isInitialized_ = false;
    key_ = 0;
    size_ = 0;
    sharedMemoryID_ = -1;
    semaphoreID_ = -1;
    sharedMemoryAddress_ = nullptr;

    // 返回清理结果
    return true;
}

// 读取共享内存
bool SharedMemory::Read(const size_t &readSize, unsigned char *readBuffer)
{
    // 判断共享内存是否初始化完毕
    if (!isInitialized_)
    {
        spdlog::error("SharedMemory must be initialized before read");
        return false;
    }

    // 判断读取长度是否合法
    if (readSize > size_)
    {
        spdlog::error("SharedMemory read out of range. key = {}", key_);
        return false;
    }

    // 获取共享内存权限
    if (!Semaphore_P())
    {
        return false;
    }

    // 从共享内存中复制数据到读取缓冲区
    memcpy(readBuffer, sharedMemoryAddress_, readSize);

    // 释放共享内存权限
    Semaphore_V();

    // 返回读取结果
    return true;
}

// 写入共享内存
bool SharedMemory::Write(const size_t &writeSize, unsigned char *writeBuffer)
{
    // 判断共享内存是否初始化完毕
    if (!isInitialized_)
    {
        spdlog::error("SharedMemory must be initialized before write");
        return false;
    }

    // 判断写入长度是否合法
    if (writeSize > size_)
    {
        spdlog::error("SharedMemory write out of range. key = {}", key_);
        return false;
    }

    // 获取共享内存权限
    if (!Semaphore_P())
    {
        return false;
    }

    // 从写入缓冲区中复制数据到共享内存
    memcpy(sharedMemoryAddress_, writeBuffer, writeSize);

    // 释放共享内存权限
    Semaphore_V();

    // 返回写入结果
    return true;
}

// 信号量的P操作
bool SharedMemory::Semaphore_P() const
{
    // 判断共享内存是否初始化完毕
    if (!isInitialized_)
    {
        spdlog::error("SharedMemory must be initialized before P-Operation");
        return false;
    }

    struct sembuf sem_buf;
    sem_buf.sem_num = 0;
    sem_buf.sem_op = -1;
    sem_buf.sem_flg = SEM_UNDO;

    if(semop(semaphoreID_, &sem_buf, 1) < 0)
    {
        spdlog::error("Semaphore‘s P-Operation execute failure. key = {}, errno = {}", key_, errno);
        return false;
    }

    // 返回操作结果
    return true;
}

// 信号量的V操作
bool SharedMemory::Semaphore_V() const
{
    // 判断共享内存是否初始化完毕
    if (!isInitialized_)
    {
        spdlog::error("SharedMemory must be initialized before V-Operation");
        return false;
    }

    struct sembuf sem_buf;
    sem_buf.sem_num = 0;
    sem_buf.sem_op = 1;
    sem_buf.sem_flg = SEM_UNDO;

    if(semop(semaphoreID_, &sem_buf, 1) < 0)
    {
        spdlog::error("Semaphore‘s V-Operation execute failure. key = {}, errno = {}", key_, errno);
        return false;
    }

    // 返回操作结果
    return true;
}