# Linux静态库/动态库/共享库

* “程序函数库”是一个包含了编译好的代码的文件，即造好的轮子
* 常用库分为三类
    * 静态库 - 事先编译进目标代码中在程序执行前就加入到程序中，后缀 .a
    * 动态链接库 - 在程序运行时动态调用，可以减小程序体积，后缀 .dll
    * 共享库 - 与 动态链接库 是同一个东西， 在linux中叫做共享库，后缀 .so 
## [编译过程](https://www.cnblogs.com/ericling/articles/11736681.html)  

* 源文件格式 - .c/.cpp/.h
* 预处理(preprocessing) - 得到.i
* 编译(compile) - 得到.s
* 汇编(assemble) - 得到.o/.obj
* 链接(link) - 得到.exe  

![编译流程](/home/lhy/Desktop/c++_library/s_vehicle.png)


## 一、静态库

### 1.静态库的制作  
1. 准备.c文件，汇编得到.o文件
2. 执行
`$ ar -rcs libxxx.a xxx.o`  
    * -r 如果不存在同名库则创建新的
    * -t 显示库文件中包含的.o文件
    * -s ？？
    * -c 表示creat，创建？？
    * 实质上是将一个或多个.o文件打包为一个.a文件
3. 执行
`$ ar -t libxx.a` 来查询静态库包含的的.o文件    
### 2.静态库的使用
`$ gcc main.c -L. -ltest`  
* 成功编译后即使删除静态库，.out文件也能正常执行，但无法重新编译
* -L 指定库文件与被编译的c文件的相对路径
* -I 指定头文件与被编译的c文件的相对路径
* -l 指定要链接的库的名字为文件名去掉lib和后缀，如库文件为libtest.a,链接时写作-ltest

## 二、动态库

### 1.动态库的制作
1. 准备.c文件，汇编得到.o文件，此处的.o不能用`$ gcc *.c -c` 来生成，否则在第2步会报错  
`$ gcc -fPIC *.c -c`
    * -fPIC - ??
2. 创建动态库  
`$ gcc -shared -o libtest.so *.o`
3. 前两步可以合成为一步  
    `$ gcc -fPIC -shared -o libxxx.so *.c`  

### 2.动态库的使用
`$ gcc main.c -L. -ltest` (与静态库相同) ， 编译得到的.out无法正常执行(因为默认会到/lib/ 或者/usr/lib/ 中寻找动态库)  
* -L 指定库文件与被编译的c文件的相对路径
* -I 指定头文件与被编译的c文件的相对路径
* -l 指定要链接的库的名字为文件名去掉lib和后缀，如库文件为libtest.a,链接时写作-ltest
* -Wl,-rpath=xx 可以指定运行时候的路径，在可执行程序运行时会优先在此路径下搜索共享库

### 3.动态库的路径搜索(按搜索优先级排序) 
1. 在编译时使用-Wl,-rpath指定路径   
2. 通过export设置环境变量LD_LIBRARY_PATH="." (表示在当前路径下搜索库)
3. 配置文件/etc/ld.so.conf中指定的动态库搜索路径
4. 搜索/usr/lib、/lib

## 三、其它
### 1.如何判断有没有链接动态库
1. file命令， 用于判断文件类型
2. ldd命令 `$ ldd x.out`

### 2.-static
g++编译时候默认动态链接，当存在静态库与动态库同名时，使用-static选项指定链接到静态库

### 3.静态动态混合链接
当需要链接的库很多时候，需要明确哪些属于静态库，哪些属于动态库  
gcc/g++优先链接动态库，若无动态库则使用静态库，或者可以使用-Wl,-Bstatic/-Wl,-Bdynamic 来指示跟在后面的-lxxx是属于动态或静态库
例如  
`$ -Wl,-Bstatic -llog4cplus -lpcap -Wl,-Bdynamic -lnl-3 -libverbs -lcurl 
`

## 四、补充说明
         在进行静态链接时一定要注意命令行中库文件放在后面，被依赖的文件也要放在后面，这里的a依赖于b是指b中定义了a中所引用的一个符号。如果文件之间依赖关系复杂，可以将 多个依赖关系复杂的文件放在一个静态库中，或者在命令行中根据依赖关系多次指明被依赖文件也是可以的，比如说x.o依赖于y.a,同时y.a依赖于z.a，并且z.a由依赖于y.a，那么此时这三个文件在命令行中的链接顺序就可以是x.o y.a z.a y.a。

        之所以这样，是因为在符号解析过程中，链接器维护一个可重定位目标文件的集合E，这个集合中的文件最终合并为可执行文件；一个未解析符号集合U，该集合中存放着被引用但是未被定义的符号；以及一个一个已经被定义的符号D，在初始状态下，三个集合均为空。所谓符号，就像变量名、函数名都是符号。

        然后链接器从左到右按照各个可重定位目标文件（.o)和静态库文件（.a）在命令行上出现的顺序来对每个文件进行扫描，如果输入文件为可重定位目标文件，那么链接器就会将该文件放到集合E中，并且将该输入文件中的符号定义和引用情况反应在集合U和集合D中；

        如果输入文件为静态库文件，那么链接器就会扫描该静态库文件中的各成员文件（.o），将成员文件中的符号与U中已被引用但是未定义的符号进行匹配，如果该静态库中某个成员文件定义了U中某个已被引用但未被定义的符号，那么链接器就将该成员文件放到集合E中，然后在U中删除该符号，D中添加该符号，再继续扫描下一个成员文件，这样一直反复进行扫描下去，直到U和D中集合都不再变化，就将多余的成员文件抛弃，然后就继续扫描下一个输入文件了。

        若文件a依赖于b，那么就说明a中存在某一符号的引用，b中存在该符号的定义。如果先扫描到符号的定义，那么就会将该符号放到D中，而由于每次扫描都是与U中的符号进行匹配，因此即使后面再扫描到该符号的引用，也会直接将该符号又放入U中，由于符号的定义已经在D中了，因此到最后该符号依然存在于U中，U中非空，说明链接中存在被引用但是未定义的符号，从而该符号被认为是未定义的符号而报错。