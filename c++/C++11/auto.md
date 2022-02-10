# 2022-01-17

## auto

* 一句话理解：是一种占位符，在编译时自动推导出类型并替代auto的位置
* 常用场景：迭代器
* 注意事项：
    1. 使用时必须立即初始化
    ```c++
    auto m;      // 错误，无法推断m的类型
    m = 12;     
    auto n = 12; // 正确 
    ```

    2. 当推导类型为引用时，auto会舍弃引用，直接推导为原数据类型
    ```c++
    int m = 12;
    int &a = m;
    auto &b = a;     // 这里的auto被推导为int类型
    auto c = b;      // 这里的auto被推导为int类型 而不是int&类型
    ```

    3. 当类型不为引用时，auto 的推导结果将不保留表达式的 const 属性
    ```c++
    int m = 12;
    const int a = m;
    auto b = a;     // 这里的auto被推导为int类型 而不是const int类型

    int m = 12;
    const int &a = m;
    auto &b = a;     // 这里的auto被推导为const int&类型, 保留了const修饰
    ```

    4. auto不能作为函数的参数使用（因为无法正确推导类型）

    5. auto不能作用于类的非静态成员对象

    6. auto不能定义数组

    7. auto不能定义模板参数

* 应用：
    1. 迭代器
    ```c++
    std::vector<int>::iterator it = v.begin();  // 冗长
    auto it = v.begin();                        // 简洁
    ```

    2. 泛型编程

    ```c++
    class A{
    public:
        static int get(void){
            return 100;
        }
    };

    class B{
    public:
        static const char* get(void){
            return "this is class B";
        }
    };

    // 使用auto自动推测返回值类型
    template<typename T>
    void fun(void){
        auto val = T::get();
        std::cout << val << std::endl;
    }

    // 在不使用auto时，需要多指定一个模板参数给返回值
    template<typename T1， typename T2>
    void fun(void){
        T2 val = T1::get();
        std::cout << val << std::endl;
    }

    int main()
    {
        fun<A>();                   // 使用auto自动推测返回值类型

        fun<A, int>();              // 指定返回值类型
        fun<B, const char*>();      

        return 0;
    }
    ```

## decltype

* 一句话描述：decltype是强化版的auto, 能够根据表达式exp推测类型

* 使用方法
`decltype(exp) value;` 
```c++
int a = 12;
decltype(a)      m_1;          // 由于使用表达式exp进行推测，因此可以不必初始化
decltype(6.3)    m_2;          // 使用常量表达式初始化，m_2类型为double
decltype(a + 12) m_3;          // 使用表达式初始化，m_3类型为int
decltype(fun())  m_4;          // 使用函数调用初始化，m_4类型为fun()的返回值
```
* 推导规则
    1. exp是一个**不被括号包围**的表达式， 或是类成员表达式， 或是一个单独的变量，那么推测结果就和exp一致
    ```c++
    decltype(12 + 24) m_1;
    decltype(A::i)    m_2;
    decltype(m)       m_3;
    ```
    2. exp是一个函数表达式, 推测结果为函数的返回值，且推导过程并不执行此函数
    ```c++
    int& fun(int r, char ch){};
    int&& fun(int r, int rr){};
    decltype(fun(1, 's')) m_1;      // m_1被推导为int&类型
    decltype(fun(1, 1))   m_2;      // m_2被推导为int&&类型
    ```
    3. exp是一个左值，或者被()包围
    ```c++
    class Base{
    public:
        int iteg;
    };
    Base obj;
    decltype(obj.iteg)   m_1;   // m_1被推测为int类型
    decltype((obj.iteg)) m_2;   // m_2被推测为int&类型

    int m = 0, n = 0;
    decltype(n + m)      m_3;   // exp为右值， m_3被推测为int
    decltype(n = n + m)  m_4;   // exp为左值， m_4被推测为int&
    ```
* 应用
    * 可以应用于类的非静态对象，查看[示例](http://c.biancheng.net/view/7151.html)