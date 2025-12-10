/**
 * @file learn_shared.cpp
 * @brief 学习 std::shared_ptr 智能指针的基本用法
 *
 * 本文件演示了 shared_ptr 的以下特性：
 * 1. 创建智能指针 (make_shared)
 * 2. 引用计数机制 (use_count)
 * 3. 获取原始指针 (get)
 * 4. 共享所有权 (拷贝赋值)
 * 5. 释放所有权 (reset)
 */

#include "iostream"
#include "memory"  // shared_ptr 所在的头文件

int main()
{
    // ========== 1. 创建智能指针 ==========
    // make_shared<T>(args...) 创建一个 shared_ptr，指向类型 T 的对象
    // 这里创建一个指向 std::string 的智能指针，内容为 "This is a str."
    auto p1 = std::make_shared<std::string>("This is a str.");

    // use_count(): 返回当前有多少个 shared_ptr 共享同一块内存
    // get(): 返回指向的原始内存地址（裸指针）
    // 此时只有 p1 指向这块内存，引用计数为 1
    std::cout << "p1的引用计数:" << p1.use_count() << ",指向的内存地址:" << p1.get() << std::endl;
    // 输出: p1的引用计数:1,指向的内存地址:0x...

    // ========== 2. 共享所有权 ==========
    // 将 p1 赋值给 p2，两者共享同一块内存
    // 这是浅拷贝，不会复制内存数据，只是增加引用计数
    auto p2 = p1;

    // 现在 p1 和 p2 都指向同一块内存，引用计数变为 2
    std::cout << "p1的引用计数:" << p1.use_count() << ",指向的内存地址:" << p1.get() << std::endl;
    std::cout << "p2的引用计数:" << p2.use_count() << ",指向的内存地址:" << p2.get() << std::endl;
    // 输出: p1的引用计数:2, p2的引用计数:2 (两者相同)
    // 输出: 两者的内存地址相同（指向同一块内存）

    // ========== 3. 释放所有权 ==========
    // reset(): 让当前 shared_ptr 放弃对内存的所有权
    // p1 不再指向任何内存，引用计数减 1
    p1.reset();

    // p1 已经被重置，不再指向任何内存
    // use_count() 返回 0，get() 返回 nullptr (显示为 0)
    std::cout << "p1的引用计数:" << p1.use_count() << ",指向的内存地址:" << p1.get() << std::endl;
    // 输出: p1的引用计数:0,指向的内存地址:0

    // p2 仍然指向原来的内存，但现在引用计数变为 1（只有 p2 在用）
    std::cout << "p2的引用计数:" << p2.use_count() << ",指向的内存地址:" << p2.get() << std::endl;
    // 输出: p2的引用计数:1,指向的内存地址:0x...（地址不变）

    // ========== 4. 访问数据 ==========
    // 使用 -> 操作符访问指向对象的成员函数
    // c_str() 是 std::string 的成员函数，返回 C 风格字符串
    std::cout << "p2指向的内存地址数据:" << p2->c_str() << std::endl;
    // 输出: p2指向的内存地址数据:This is a str.

    // ========== 5. 自动释放内存 ==========
    // 当 main 函数结束时，p2 离开作用域
    // p2 的引用计数变为 0，内存自动释放
    // 无需手动 delete，避免内存泄漏！
}