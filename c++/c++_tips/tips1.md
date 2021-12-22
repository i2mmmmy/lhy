# 预编译指令
* `#define` 宏定义
* `#if/#endif、#ifdef/#endif` 条件编译
    * 常用来注释一整段代码
    ```c++
    #if 0
    void hello(){};
    #endif
    ```
* #运算符
    * `#define MKSTR(x) #x // 将x转换为字符串`
        ```c++
        #define MKSTR(x) #x
        std::cout << MKSTR(abc) <<std::endl;
        // output:  "abc"        
        ```
    * `#define CONTACT(x,y) x##y // 将x和y连接起来`.
        ```c++
        #define CONTACT(x,y) x##y
        std::cout << CONTACT(ab,c,d,ee,a) <<std::endl;
        // output:  "abcdeea"        
        ```   


# 左值和右值

# 拷贝构造

# 传入常引用的作用(const int &a)

1. 函数传入引用，在函数内可以改变这个传入值
    ```c++
    void ref_fun(int &a)
    {
        a *= 2;
    }

    // void ref_fun(int a) // 函数声明中引用和非引用不能重载

    void fun(int a)
    {
        a *= 2;
    }

    int main()
    {
        int a = 1;
        std::cout << "before: " << a << std::endl;
        ref_fun(a);
        std::cout << "after: " << a << std::endl;
    }
    ```
2. 非引用的形参，在函数使用时是将实参复制一份传入函数进行操作
    * 当传入的实参很大时例如`std::vector<int>`这种可能会很大的数组，复制会非常浪费时间，因此使用引用传入能节约时间
    * 使用引用传入能节约时间但无法保证传入数据不被函数意外修改，因此使用常引用
    ```c++
    void conref_fun(const std::vector<int> &vec);
    ```
    可以兼顾快速与安全

# nodehandle.h  252行？

# 函数后的关键字
https://blog.csdn.net/u011559046/article/details/120805223

# this指针
* 是一个在类的内部指向类自身的指针
* 每个对象有自己的this指针
* 类的非静态成员函数对自己类内非静态成员进行操作时，系统隐式地使用了this指针

# 多线程

# 类的静态成员变量 static

# 模板类的多态？？？

# #pragma once

# [ERROR] [1635990487.627053144]: client wants service /winhye_ugv/set_mode_state to have md5sum 939409827c5731ec5346c99677bd2a1d, but it has eafa91a8697395903cb69878c720467a. Dropping connection.
* 客户端消息类型和服务消息类型不匹配