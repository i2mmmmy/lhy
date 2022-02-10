# GDB调试

## 在普通.c/.cpp工程中使用gdb

`(必要步骤)`编译时使用`-g`参数，生成可供调试的可执行程序  
* `$ g++ -g main.cpp`


## 在ros中使用gdb

1. `(必要步骤)`编译出可供调试的debug程序  
    编译时使用  
    `$ catkin_make -DCMAKE_BUILD_TYPE=Debug`  
    或  
    `$ catkin build -DCMAKE_BUILD_TYPE=Debug`  
2. 单个节点运行时，使用以下指令  
    * 使用rosrun参数运行节点  
    `$ rosrun --prefix 'gdb -q -ex --args' ${PKG_NAME} ${TYPE_NAME}`  
    * 直接使用gdb的file指令指向存放在`workspace/devel/lib/${PKG_NAME}/`目录下的可执行文件
3. (需要安装xterm) launch文件中启动多个节点时，在需要调试的节点的<node />中添加prefix参数如下  
    `<node pkg="learning_tf" type="testlib" name="sim" launch-prefix="xterm -e gdb -q -ex --args"/>`
    `<node pkg="learning_tf" type="testlib" name="sim" launch-prefix="gnome-terminal -- gdb -q -ex --args"/>`

## gdb简单使用

1. 在终端中输入gdb打开gdb工具，推荐使用`-q(quiet)`参数来隐藏多余的版本说明信息  
    `gdb -q`
2. 使用`file`指令来定向一个可调试的可执行程序  
    `(gdb) file ${file_path}/${exec_name}`  
    `(gdb) file ./a.out`
3. 使用`s`(start)指令，使得程序执行到main()函数第一行并暂停  
    `(gdb) s`
4. 使用`b`(break)指令，在指定行设置断点breakpoint  
    ```shell
    (gdb) b ${__line__}  
    (gdb) b 12          # 在第12行设置断点
    (gdb) b +1          # 在当前行号+1行打断点 
    (gdb) b 3 if exp    # 若exp成立，则在第三行打断点(用于调试循环十分有效)
    ```
    * 其他断点指令(替代break)
        * tbreak - 只生效一次的断点
        * rbreak - rbreak regex, 其中regex为正则表达式，程序中符合此正则的函数名都会在函数开始位置打上断点
5. 使用`l`(list)指令，显示该可执行程序的代码  
    `(gdb) l`  