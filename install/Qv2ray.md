# Qv2ray安装，配置

## 准备

1. 下载[客户端](https://github.com/Qv2ray/Qv2ray/releases/tag/v2.7.0)  
2. 下载[v2ray核心库](https://github.com/v2fly/v2ray-core/releases)  

## 配置流程

1. 给客户端加权限  
2. 打开客户端-首选项，设置v2ray可执行文件路径和资源目录  
3. 打开客户端-分组，在订阅设置中输入订阅地址，更新订阅  

## 使用
1. 在浏览器使用代理
    * 下载chrome浏览器的扩展程序[switchyomega]](https://github.com/FelisCatus/SwitchyOmega/releases)(已有此插件可忽略)
    * 将下载的.crx文件后缀改为.zip,然后解压到本地的文件夹中，文件夹命名为switchyomega
    * 打开chrome浏览器-设置-扩展程序-开发者模式，将文件夹拖入
    * 根据qv2ray客户端设置，配置扩展程序switchyomega
    * 或者可以直接使用switchyomega的系统代理选项
    
2. 在terminal终端使用代理
    * 安装proxychains
    ```bash
    $ sudo apt install proxychains
    $ sudo apt install proxychains4 # 不知道有什么区别，这俩都装上吧
    ```
    * 编辑`/etc/proxychains.conf`，根据qv2ray客户端设置，在该文件的最后一行添加上代理信息
        * 比如在配置最后一行添加`socks5 127.0.0.1 1088`

    * 使用时打开终端， 在需要代理的命令前加上 `proxychains4`
        * 测试：
        ```bash
        $ proxychains4 curl www.google.com
        ```