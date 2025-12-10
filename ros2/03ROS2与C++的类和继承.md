# 03 ROS2 与 C++ 的类和继承

---

## 一、C++ 类与 ROS2 的连接机制

### 1.1 整体架构图

```
┌─────────────────────────────────────────────────────────┐
│                    ROS2 系统                            │
│  ┌─────────────────────────────────────────────────┐   │
│  │              rclcpp::Node (基类)                 │   │
│  │  ┌─────────────────────────────────────────┐   │   │
│  │  │  - 节点名称管理                          │   │   │
│  │  │  - 日志系统 (get_logger)                 │   │   │
│  │  │  - 话题发布/订阅                         │   │   │
│  │  │  - 服务端/客户端                         │   │   │
│  │  │  - 参数管理                              │   │   │
│  │  │  - 定时器                                │   │   │
│  │  └─────────────────────────────────────────┘   │   │
│  └─────────────────────────────────────────────────┘   │
│                         ▲                               │
│                         │ 继承 (inheritance)            │
│                         │                               │
│  ┌─────────────────────────────────────────────────┐   │
│  │           PersonNode (你的自定义类)              │   │
│  │  ┌─────────────────────────────────────────┐   │   │
│  │  │  - name_ (自定义属性)                    │   │   │
│  │  │  - age_  (自定义属性)                    │   │   │
│  │  │  - eat() (自定义方法)                    │   │   │
│  │  │  + 继承的所有Node功能                    │   │   │
│  │  └─────────────────────────────────────────┘   │   │
│  └─────────────────────────────────────────────────┘   │
└─────────────────────────────────────────────────────────┘
```

### 1.2 示例代码

```cpp
#include "rclcpp/rclcpp.hpp"

// 自定义节点类，继承自 rclcpp::Node
class PersonNode : public rclcpp::Node
{
private:
    std::string name_;  // 人的名字（自定义属性）
    int age_;           // 人的年龄（自定义属性）

public:
    // 构造函数：node_name传给父类，name和age是自己的属性
    PersonNode(const std::string &node_name, const std::string &name, const int &age)
        : Node(node_name)  // 调用父类构造函数，注册节点名
    {
        this->name_ = name;
        this->age_ = age;
    };

    // 自定义方法：使用继承来的 get_logger()
    void eat(const std::string &food_name)
    {
        RCLCPP_INFO(this->get_logger(), "我是%s，%d岁，爱吃%s",
                    this->name_.c_str(), this->age_, food_name.c_str());
    };
};

int main(int argc, const char **argv)
{
    // ① 初始化ROS2运行时环境
    rclcpp::init(argc, argv);

    // ② 创建节点实例（智能指针）
    // "person_node" 是节点名，"张三"和25是自定义属性
    auto node = std::make_shared<PersonNode>("person_node", "张三", 25);

    // ③ 使用节点功能
    RCLCPP_INFO(node->get_logger(), "hello world");
    node->eat("ros");

    // ④ 让节点保持运行，等待回调
    rclcpp::spin(node);

    // ⑤ 关闭ROS2
    rclcpp::shutdown();

    return 0;
}
```

### 1.3 继承声明（核心连接点）

```cpp
class PersonNode : public rclcpp::Node
//    ↑              ↑         ↑
//    你的类名    公有继承   ROS2节点基类
```

**这行代码的含义**：
- `PersonNode` 成为 `rclcpp::Node` 的**子类**
- `PersonNode` 自动获得 `rclcpp::Node` 的所有功能
- 这就像"继承家产"：父类有的，子类都能用

**形象比喻**：
```
rclcpp::Node = ROS2世界的"公民身份证"
继承它       = 你的类"入籍"成为ROS2公民
入籍后       = 可以合法使用ROS2的所有基础设施（日志、话题、服务等）
```

### 1.4 构造函数的初始化列表

```cpp
PersonNode(const std::string &node_name, const std::string &name, const int &age)
    : Node(node_name)  // ← 关键！调用父类构造函数
{
    this->name_ = name;
    this->age_ = age;
};
```

**执行顺序**：
```
步骤1: Node(node_name)    → 先初始化父类，向ROS2注册节点名
步骤2: this->name_ = name → 再初始化子类的属性
步骤3: this->age_ = age   → 继续初始化子类的属性
```

**为什么必须调用 `Node(node_name)`？**
- `rclcpp::Node` 的构造函数需要一个节点名称
- 这个名称会被ROS2系统用来识别和管理你的节点
- 不调用就无法完成"入籍"

### 1.5 使用继承来的功能

```cpp
void eat(const std::string &food_name)
{
    // get_logger() 是从 rclcpp::Node 继承来的方法
    RCLCPP_INFO(this->get_logger(), "我是%s，%d岁，爱吃%s", ...);
}
```

**`this->get_logger()` 的来源**：
```
PersonNode
    └── 继承自 rclcpp::Node
                └── 提供了 get_logger() 方法
```

你没有写 `get_logger()`，但能用它，因为**继承**让你拥有了父类的所有公有方法。

---

## 二、ROS1 与 ROS2 编程风格对比

### 2.1 为什么 ROS1 不需要类与继承？

ROS1 和 ROS2 采用了完全不同的编程范式：

| 对比项 | ROS1 | ROS2 |
|--------|------|------|
| **设计理念** | 快速上手，简单直接 | 工业级，规范严谨 |
| **编程风格** | 函数式 | 面向对象（OOP） |
| **节点是什么** | 一个进程 + NodeHandle | 一个类的实例 |
| **获取功能的方式** | 通过 `NodeHandle` 句柄 | 通过 `继承 Node` |
| **入门门槛** | 低（会写 main 就行） | 高（需要懂类、继承） |

### 2.2 代码对比：同样功能的节点

#### ROS1 写法（函数式，不需要类）

```cpp
#include "ros/ros.h"

// ROS1：直接在 main 里写，不需要定义类
int main(int argc, char **argv)
{
    ros::init(argc, argv, "person_node");  // 初始化 + 节点名
    ros::NodeHandle nh;                     // 获取句柄（不是继承！）

    ROS_INFO("我是张三，25岁，爱吃ros");     // 直接用

    ros::spin();
    return 0;
}
```

#### ROS2 写法（面向对象，需要类）

```cpp
#include "rclcpp/rclcpp.hpp"

// ROS2：必须定义一个类，继承 rclcpp::Node
class PersonNode : public rclcpp::Node
{
public:
    PersonNode() : Node("person_node") {}  // 构造函数里注册节点名
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<PersonNode>();  // 创建类实例
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
```

### 2.3 设计思路图示

```
【ROS1 的思路】—— "给你一把钥匙"
┌─────────────┐
│   main()    │
│      │      │
│      ▼      │
│ NodeHandle  │ ← 这是一把"钥匙"（句柄）
│      │      │    拿着它就能用 ROS 功能
│      ▼      │
│  用钥匙开门  │    nh.advertise(), nh.subscribe()...
└─────────────┘

【ROS2 的思路】—— "你自己变成 ROS 公民"
┌─────────────┐
│  YourNode   │
│      │      │
│   继承 ↓    │
│ rclcpp::Node│ ← 你自己"变成"了节点
│      │      │    天生就有 ROS 功能
│      ▼      │
│  直接使用    │    this->create_publisher()...
└─────────────┘
```

### 2.4 为什么 ROS2 要改成面向对象？

| 原因 | 说明 |
|------|------|
| **更好的封装** | 节点的状态、回调都在类里，不会散落各处 |
| **支持组件化** | 多个节点可以运行在同一个进程里（composition） |
| **生命周期管理** | 节点可以有状态（未配置→配置→激活→停用） |
| **更适合大型项目** | 工业机器人、自动驾驶需要规范的代码结构 |

### 2.5 简单总结

```
ROS1: "这是工具箱（NodeHandle），你拿去用"
      → 简单，但代码容易乱

ROS2: "你自己变成机器人的一部分（继承Node）"
      → 规范，但需要学 C++ 面向对象
```

**结论**：
- ROS1 入门简单，因为不需要懂类和继承
- ROS2 入门难一点，但代码更规范、更适合工业应用
- 学习 C++ 类与继承，是掌握 ROS2 的**必备知识**

---

## 三、main 函数执行流程

```cpp
int main(int argc, const char **argv)
{
    rclcpp::init(argc, argv);                                          // ① 启动ROS2引擎
    auto node = std::make_shared<PersonNode>("person_node","张三",25);  // ② 创建节点
    RCLCPP_INFO(node->get_logger(), "hello world");                    // ③ 使用功能
    node->eat("ros");                                                  // ③ 使用功能
    rclcpp::spin(node);                                                // ④ 阻塞等待
    rclcpp::shutdown();                                                // ⑤ 关闭ROS2
    return 0;
}
```

**流程图**：
```
┌──────────────────┐
│  rclcpp::init()  │  ← 启动ROS2引擎
└────────┬─────────┘
         ▼
┌──────────────────┐
│  make_shared<>   │  ← 创建PersonNode对象
│  PersonNode()    │     同时完成Node初始化
└────────┬─────────┘
         ▼
┌──────────────────┐
│  使用节点功能     │  ← 日志输出、调用eat()
└────────┬─────────┘
         ▼
┌──────────────────┐
│  rclcpp::spin()  │  ← 节点进入"待命"状态
│  (阻塞等待)       │     处理话题/服务/定时器回调
└────────┬─────────┘
         ▼
┌──────────────────┐
│ rclcpp::shutdown │  ← 清理资源，退出ROS2
└──────────────────┘
```

---

## 四、构造函数参数解析

### 4.1 哪个是节点名？

```cpp
auto node = std::make_shared<PersonNode>("person_node", "张三", 25);
//                                           ↑           ↑      ↑
//                                       节点名称    自定义属性  自定义属性
//                                       (传给ROS2)  (存在类里)  (存在类里)
```

| 参数 | 值 | 传给谁 | 作用 |
|------|-----|--------|------|
| `node_name` | `"person_node"` | 父类 `Node()` | ✅ **ROS2节点名称** |
| `name` | `"张三"` | `this->name_` | ❌ 只是普通属性 |
| `age` | `25` | `this->age_` | ❌ 只是普通属性 |

### 4.2 参数流向图

```
"person_node"  ──→  Node(node_name)  ──→  ROS2系统注册  ✅ 这是节点名
"张三"         ──→  this->name_      ──→  类的私有变量  ❌ 普通数据
25            ──→  this->age_       ──→  类的私有变量  ❌ 普通数据
```

### 4.3 验证方法

运行节点后，在另一个终端执行：

```bash
ros2 node list
```

输出：
```
/person_node    ← 只显示这个，不会显示"张三"或"25"
```

**结论**：`"张三"` 和 `25` 只是自定义数据，与 ROS2 无关；只有 `"person_node"` 是真正注册到 ROS2 系统中的节点名称。

---

## 五、C++ 语法要点

### 5.1 为什么用 `->` 而不是 `.`？

```cpp
auto node = std::make_shared<PersonNode>(...);  // node 是智能指针
node->eat("ros");                               // 指针用 ->
```

#### 核心区别

| 符号 | 用于 | 示例 |
|------|------|------|
| `.` | 普通对象 | `obj.method()` |
| `->` | 指针 | `ptr->method()` |

#### 对比示例

```cpp
// 方式1：普通对象（栈上）—— 用点号
PersonNode obj("node1", "张三", 25);
obj.eat("ros");      // 用 .

// 方式2：原始指针（堆上）—— 用箭头
PersonNode* ptr = new PersonNode("node2", "李四", 30);
ptr->eat("ros");     // 用 ->

// 方式3：智能指针（ROS2常用）—— 用箭头
auto node = std::make_shared<PersonNode>("node3", "王五", 35);
node->eat("ros");    // 用 ->  （shared_ptr 也是指针）
```

**记忆口诀**：
> 指针用箭头 `->`，对象用点 `.`

### 5.2 为什么参数要加 `&`（引用）？

```cpp
PersonNode(const std::string &node_name, const std::string &name, const int &age)
//                          ↑                            ↑               ↑
//                         引用                         引用            引用
```

#### `&` 是"引用"，作用是**避免拷贝**

| 写法 | 传递方式 | 效率 |
|------|----------|------|
| `std::string name` | 拷贝整个字符串 | ❌ 慢（复制数据） |
| `std::string &name` | 传地址，不拷贝 | ✅ 快 |
| `const std::string &name` | 传地址 + 不可修改 | ✅ 快且安全 |

#### 图示对比

```
【不用引用】值传递 - 复制一份
┌─────────┐    复制    ┌─────────┐
│ "张三"   │  ──────→  │ "张三"   │  ← 新的副本，浪费内存
└─────────┘           └─────────┘
  原始数据              函数内的副本

【用引用】引用传递 - 直接访问原数据
┌─────────┐
│ "张三"   │  ←──────── 函数直接访问这里，不复制
└─────────┘
  原始数据
```

#### 为什么加 `const`？

```cpp
const std::string &name   // 承诺：只读不改
```

- **保护数据**：函数内部不能修改原始值
- **表明意图**：告诉调用者"我不会动你的数据"

#### 特例：`int` 其实不需要引用

```cpp
const int &age   // 可以，但没必要
int age          // 这样更简单，int 很小，拷贝无所谓
```

`int` 只有 4 字节，拷贝比引用还快。但写 `const int &` 也没错，只是习惯问题。

---

## 六、为什么用 `std::make_shared`？

```cpp
// ROS2 推荐写法
auto node = std::make_shared<PersonNode>("person_node", "张三", 25);
```

**原因**：
1. `rclcpp::spin()` 要求传入 `std::shared_ptr` 类型
2. ROS2内部需要共享节点的所有权
3. 智能指针自动管理内存，防止泄漏

**等价的写法**（不推荐）：

```cpp
// 不推荐：写法繁琐
std::shared_ptr<PersonNode> node(new PersonNode("person_node", "张三", 25));
```

---

## 七、总结

### 7.1 连接的本质

| 层面 | C++ 机制 | ROS2 作用 |
|------|----------|-----------|
| 连接方式 | `class A : public rclcpp::Node` | 让自定义类成为ROS2节点 |
| 初始化 | 构造函数调用 `Node(name)` | 向ROS2注册节点 |
| 功能获取 | 继承 + `this->` | 使用日志、话题、服务等 |
| 生命周期 | `shared_ptr` + `spin` | 被ROS2系统管理 |

### 7.2 一句话总结

> C++ 通过**继承** `rclcpp::Node` 类，让你的自定义类获得"ROS2公民身份"，从而可以使用ROS2提供的所有通信和管理功能。

### 7.3 C++ 语法速查

| 问题 | 答案 |
|------|------|
| 为什么用 `->` | 因为 `node` 是智能指针，指针访问成员必须用 `->` |
| 为什么用 `&` | 引用传递，避免拷贝大对象（如 string），提高效率 |
| 为什么加 `const` | 保护原始数据不被修改，更安全 |

---

## 学习资源

| 资源 | 链接 |
|------|------|
| ROS2 Humble 官方中文文档 | https://ros2docs.robook.org/humble/Tutorials/Beginner-Client-Libraries/Writing-A-Simple-Cpp-Publisher-And-Subscriber.html |
| 鱼香ROS 中文教程 | http://dev.ros2.fishros.com/doc/Tutorials/Writing-A-Simple-Cpp-Publisher-And-Subscriber.html |
| CSDN 笔记 | https://blog.csdn.net/weixin_71160458/article/details/138162800 |
