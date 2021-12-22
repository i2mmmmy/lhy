# paho_mqtt安装

## 准备

1. 执行以下语句安装依赖
```bash
$ sudo apt-get install build-essential gcc make cmake cmake-gui cmake-curses-gui
$ sudo apt-get install libssl-dev 
$ sudo apt-get install doxygen graphviz
```
2. (可选)安装[catch2](https://github.com/catchorg/Catch2)

## 安装paho.c

```bash
$ git clone https://github.com/eclipse/paho.mqtt.c.git #速度太慢可翻墙
$ cd paho.mqtt.c
$ git checkout v1.3.8
$ cmake -Bbuild -H. -DPAHO_ENABLE_TESTING=OFF -DPAHO_BUILD_STATIC=ON -DPAHO_WITH_SSL=ON -DPAHO_HIGH_PERFORMANCE=ON
$ sudo cmake --build build/ --target install
$ sudo ldconfig
```

## 安装paho.cpp
```bash
$ git clone https://github.com/eclipse/paho.mqtt.cpp #速度太慢可翻墙
$ cd paho.mqtt.cpp
$ cmake -Bbuild -H. -DPAHO_BUILD_STATIC=ON -DPAHO_BUILD_DOCUMENTATION=TRUE -DPAHO_BUILD_SAMPLES=TRUE
$ sudo cmake --build build/ --target install
$ sudo ldconfig
```