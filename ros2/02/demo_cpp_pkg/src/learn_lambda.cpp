/**
 * @file learn_lambda.cpp
 * @brief 学习 C++11 Lambda 表达式的基本用法
 *
 * Lambda 表达式语法：
 * [捕获列表](参数列表) -> 返回类型 { 函数体 }
 *
 * 捕获方式：
 * []     - 不捕获任何外部变量
 * [x]    - 值捕获：复制 x 的值
 * [&x]   - 引用捕获：引用 x，可修改原变量
 * [=]    - 值捕获所有外部变量
 * [&]    - 引用捕获所有外部变量
 * [this] - 捕获当前对象指针（类成员函数中使用）
 */

#include "iostream"
#include "algorithm"

int main()
{
    // ========== Lambda 1: 带参数的 lambda ==========
    // 定义一个计算两数之和的 lambda 表达式
    auto add = [](int a, int b) -> int
    {
        return a + b;
    };
    // 解析：
    // []        : 捕获列表为空，不捕获任何外部变量
    // (int a, int b) : 参数列表，接收两个 int 类型参数
    // -> int    : 返回类型为 int（可省略，编译器会自动推导）
    // { return a+b; } : 函数体，返回 a+b 的结果

    // 调用 lambda，就像调用普通函数一样
    int sum = add(200, 100);  // sum = 300

    // ========== Lambda 2: 捕获外部变量的 lambda ==========
    // 定义一个打印 sum 值的 lambda 表达式
    auto print_sum = [sum]() -> void
    {
        std::cout << sum << std::endl;
    };
    // 解析：
    // [sum]  : 值捕获，将外部变量 sum 的值（300）复制到 lambda 内部
    // ()     : 参数列表为空，无参数
    // -> void: 返回类型为 void（无返回值，可省略）
    // 注意：值捕获是复制，即使外部 sum 改变，lambda 内的值不变

    // 调用 lambda，输出 300
    print_sum();

    // ========== 补充：引用捕获示例 ==========
    // int x = 10;
    // auto modify_x = [&x]() { x = 20; };  // 引用捕获，可修改原变量
    // modify_x();  // 执行后 x 变为 20

    return 0;
}
