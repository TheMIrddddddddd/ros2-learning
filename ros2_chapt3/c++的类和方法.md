# C++ 的类和方法

## 1. 类 (Class) vs 方法 (Method)

**类 (Class)** 是一个"设计图纸"，定义了对象有什么属性和行为：

```cpp
class Cake {           // 这是一个"类"
    std::string flavor;  // 属性：口味
    int layers;          // 属性：层数

    void eat();          // 方法：吃掉它
    void decorate();     // 方法：装饰它
};
```

**方法 (Method)** 是类里面的"动作"函数：

```cpp
void Cake::eat() {     // 这是一个"方法"，定义了具体怎么做
    std::cout << "好吃！" << std::endl;
}
```

### 简单区分

| 概念 | 是什么 | 怎么认 |
|------|--------|--------|
| **类** | 名词，是"东西" | `class Xxx { }` |
| **方法** | 动词，是"动作" | 类里面的函数 `返回类型 函数名()` |

---

## 2. 成员函数 vs 方法（术语）

在C++里，更准确的叫法是**成员函数 (Member Function)**，但叫"方法"也可以。

```cpp
class Guitar {
public:
    void play() { }        // 普通成员函数（方法）
    void tune() { }        // 普通成员函数（方法）

    Guitar() { }           // 构造函数（特殊的成员函数）
    ~Guitar() { }          // 析构函数（特殊的成员函数）
};

// 这个在类外面，叫"普通函数"或"自由函数"
void singASong() { }
```

| 位置 | C++官方叫法 | 通俗叫法 |
|------|------------|---------|
| 类**里面** | 成员函数 | 方法 |
| 类**外面** | 函数 / 自由函数 | 函数 |

---

## 3. 声明和定义分离

可以在类里面只写**声明**，在类外面写**定义**：

```cpp
// ===== 类里面只是"声明" =====
class Guitar {
public:
    Guitar();           // 声明：我有一个构造函数
    ~Guitar();          // 声明：我有一个析构函数
    void play();        // 声明：我会弹奏
private:
    std::string name_;
};

// ===== 类外面写"定义" =====
Guitar::Guitar() {                    // Guitar:: 表示"这是Guitar类的"
    name_ = "吉太";
    std::cout << "吉他诞生了！" << std::endl;
}

Guitar::~Guitar() {
    std::cout << "吉他说拜拜~" << std::endl;
}

void Guitar::play() {
    std::cout << "铛铛铛~" << std::endl;
}
```

### 使用时的行为

不管定义写在类里面还是外面，它们都是类的成员函数，行为完全一样：

```cpp
int main() {
    Guitar myGuitar;      // 自动调用构造函数 Guitar::Guitar()
    myGuitar.play();      // 调用 Guitar::play()
    return 0;
}                         // myGuitar 离开作用域，自动调用析构函数
```

### 为什么要分开写？

| 写法 | 适合场景 |
|------|---------|
| 类里面直接写 | 函数很短、很简单 |
| 类外面分开写 | 函数很长、逻辑复杂 |

分开写可以让类的结构更清晰：声明放前面一目了然，定义放后面详细实现。
