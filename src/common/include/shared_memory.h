//
// Created by plutoli on 2021/6/13.
//

#ifndef CUBOT_BRAIN_SHARED_MEMORY_H
#define CUBOT_BRAIN_SHARED_MEMORY_H

#include <sys/types.h>
#include <sys/ipc.h>
#include <sys/shm.h>
#include <sys/sem.h>
#include "spdlog/spdlog.h"

#define ACCESS_BIT 0666

/**
 * @brief 基于IPC中的共享内存和信号量的通用共享内存类
 * @details 参考网址：\n
 *      https://blog.csdn.net/weixin_43055404/article/details/107936481 \n
 *      https://www.cnblogs.com/wucongzhou/p/12497864.html \n
 *      https://blog.csdn.net/zjy900507/article/details/106219635 \n
 *      https://blog.csdn.net/dove1202ly/article/details/79999540 \n
 *      https://blog.csdn.net/qq_22075977/article/details/77973186 \n
 *      https://www.cnblogs.com/lchb/articles/5570428.html \n
  */
class SharedMemory
{
public:
    /**
     * @brief 构造函数，初始化SharedMemory的实例
     */
    SharedMemory();

    /**
     * @brief 析构函数，注销SharedMemory的实例
     */
    ~SharedMemory() = default;

    /**
     * @brief 拷贝构造函数
     * @param[in] sharedMemory  拷贝对象
     * @note 禁用拷贝构造函数
     */
    SharedMemory(const SharedMemory &sharedMemory) = delete;

    /**
     * @brief 拷贝赋值运算符
     * @param[in] sharedMemory  拷贝对象
     * @return 拷贝赋值结果
     * @note 禁用拷贝赋值运算符
     */
    SharedMemory& operator=(const SharedMemory &sharedMemory) = delete;

    /**
     * @brief 移动构造函数
     * @param[in] sharedMemory  移动对象
     * @note 禁用移动构造函数
     */
    SharedMemory(SharedMemory &&sharedMemory) = delete;

    /**
     * @brief 移动赋值运算符
     * @param[in] sharedMemory  移动对象
     * @return 移动赋值结果
     * @note 禁用移动赋值运算符
     */
    SharedMemory& operator=(SharedMemory &&sharedMemory) = delete;

    /**
     * @brief 获取共享内存的初始化状态
     * @return 初始化状态\n
     *         -<em>false</em> 初始化失败\n
     *         -<em>true</em> 初始化成功\n
     */
    bool GetStatus() const;

    /**
     * @brief 获取共享内存和数据同步信号量的键
     * @note 共享内存和数据同步信号量的使用相同的键
     * @return 共享内存和数据同步信号量的键
     */
    key_t GetKey() const;

    /**
     * @brief 获取共享内存的字节长度
     * @return 共享内存的字节长度
     */
    size_t GetSize() const;

    /**
     * @brief 初始化共享内存
     * @note 程序在初始化过程中，将会创建或挂载共享内存，创建或挂载用于数据进程同步的信号量
     * @param[in] key   共享内存和信号量的键，可以使用ftok()函数的返回值，也可以自行输入一个key_t类型的数值
     * @param[in] size  共享内存的大小
     * @return 共享内存初始化是否成功\n
     *         -<em>false</em> 初始化失败\n
     *         -<em>true</em> 初始化成功\n
     */
    bool Init(const key_t &key, const size_t &size);

    /**
     * @brief 清理共享内存资源
     * @note 程序在清理过程中，将会删除共享内存和数据同步的信号量
     * @return 共享内存资源清理是否成功\n
     *         -<em>false</em> 清理失败\n
     *         -<em>true</em> 清理成功\n
     */
    bool Clear();

    /**
     * @brief 从共享内存中读取以字节为单位指定长度的数据
     * @note 在读取之前，必须先初始化共享内存；同时readSize不能大于共享内存的大小
     * @param[in] readSize      读取长度
     * @param[out] readBuffer   读取缓冲区
     * @return 读取操作是否成功\n
     *         -<em>false</em> 读取失败\n
     *         -<em>true</em> 读取成功\n
     */
    bool Read(const size_t &readSize, unsigned char *readBuffer);

    /**
     * @brief 将以字节为单位指定长度的数据写入共享内存
     * @note writeSize不能大于共享内存的大小
     * @param[in] writeSize     写入长度
     * @param[in] writeBuffer   写入缓冲区
     * @return 写入操作是否成功\n
     *         -<em>false</em> 写入失败\n
     *         -<em>true</em> 写入成功\n
     */
    bool Write(const size_t &writeSize, unsigned char *writeBuffer);

private:
    bool isInitialized_;                  ///< 共享内存的初始化标志
    key_t key_;                           ///< 共享内存和数据同步信号量的键
    size_t size_;                         ///< 共享内存的字节长度
    int sharedMemoryID_;                  ///< 共享内存的ID
    int semaphoreID_;                     ///< 信号量的ID
    unsigned char *sharedMemoryAddress_;  ///< 共享内存在当前进程地址空间中的地址

    /**
     * @brief 信号量的P操作
     * @note 如果执行P操作之前共享内存没有初始化或者数据同步信号量已经被删除，会导致操作失败
     * @return 信号量的P操作是否成功\n
     *         -<em>false</em> P操作失败\n
     *         -<em>true</em> P操作成功\n
     */
    bool Semaphore_P() const;

    /**
     * @brief 信号量的V操作
     * @note 如果执行V操作之前共享内存没有初始化或者数据同步信号量已经被删除，会导致操作失败
     * @return 信号量的V操作是否成功\n
     *         -<em>false</em> V操作失败\n
     *         -<em>true</em> V操作成功\n
     */
    bool Semaphore_V() const;
};

/**
 * @brief semctl()函数使用的联合体
 */
union semun
{
    int val;                ///< 传递信号量的初始值
    struct semid_ds *buf;   ///< 一般使用不到
    unsigned short *array;  ///< 一般使用不到
};

#endif //CUBOT_BRAIN_SHARED_MEMORY_H