# TF

## broadcaster

### 成员介绍
* `tf::TransformBroadcaster`类， 用于向tf树广播一个节点
    * 有一个默认构造
    * 有一个方法sendTransform()，该方法有4个重载
    ```c++
    /** \brief 发送一个 StampedTransform(带标签的转换)
    * 该转换包含 frame_id, time, parent_id.  */
    void sendTransform(const StampedTransform & transform);

    /** \brief 发送一个 StampedTransform向量
    * 该转换包含 frame_id, time, parent_id.  */
    void sendTransform(const std::vector<StampedTransform> & transforms);

    /** \brief 发送一个 geometry_msg::TransformStamped类型的转换
    * 该转换包含 frame_id, time, parent_id.  */
    void sendTransform(const geometry_msgs::TransformStamped & transform);

    /** \brief 发送一个 geometry_msg::TransformStamped转换的向量
    * 该转换包含 frame_id, time, parent_id.  */
    void sendTransform(const std::vector<geometry_msgs::TransformStamped> & transforms);
    ```
* `tf::StampedTransform`类，用于记录一个tf节点的信息，继承`tf::Transform`类
    * 类成员包括时间戳、 父节点id、 子节点id
    * 构造函数使用传入参数初始化类成员
        ```c++
        StampedTransform(const tf::Transform& input, const ros::Time& timestamp, const std::string & frame_id, const std::string & child_frame_id):
        tf::Transform (input), stamp_ ( timestamp ), frame_id_ (frame_id), child_frame_id_(child_frame_id){ }
        ```
    * 成员函数
        * `void setData(const tf::Transform& input);`
            * 用于设置它继承的`tf::Transform`类中的数据
* `tf::Transform`类， 记录tf转换的信息，该类只支持刚性转换(**坐标旋转和坐标位移**)，但不包括坐标缩放和裁剪(应该是指坐标区域分割)
    * 有三个构造函数
        1. 默认构造
        2. **四元数(用来描述旋转) + vector3(用来描述平移)** 
        3. **使用3*3矩阵(用来描述旋转) + vector3(用来描述平移)** 
    * 常用的几个方法
        1. 设置类成员`Vector3   m_origin;`的数值  
            `void setOrigin(const Vector3& origin);`
        2. 使用**3*3矩阵**初始化类成员`Matrix3x3 m_basis;`  
            `void setBasis(const Matrix3x3& basis);`
        3. 使用**四元数**初始化类成员`Matrix3x3 m_basis;`  
            `void setRotation(const Quaternion& q);`
### 使用方法：
1. 生成一个`tf::Transform`对象并初始化平移和旋转  
    ```c++
    tf::Transform transform;
    // 设置tf坐标位移
    transform.setOrigin( tf::Vector3(msg->x, msg->y, 0.0) );
    // 初始化一个四元数来记录旋转信息
    tf::Quaternion q;
    // 使用rpy(roll, pitch, yaw)格式来记录旋转信息
    q.setRPY(0, 0, msg->theta);
    // 设置tf坐标旋转
    transform.setRotation(q);
    ```
2. 生成一个`tf::StampedTransform`对象并初始化转换信息、时间戳、父节点、子节点  
    ```c++
    // 使用步骤1.生成的transform，转换时间为当前时间，父节点为"world", 子节点为turtle_name
    tf::StampedTransform stamp(transform, ros::Time::now(), "world", turtle_name);
    ```
3. 生成一个`tf::TransformBroadcaster`对象并发送一次tf转换信息
    ```c++
    static tf::TransformBroadcaster br;
    // 将步骤2.生成的stamp发送出去
    br.sendTransform(stamp);
    ```
4. `rosrun tf tf_echo /world /turtle1` ，显示 **/turtle1坐标系** 相对于 **/world坐标系** 的位置
## listener

### 成员介绍
* 类 `tf::TransformListener` ，用于从tf树上获取两个节点的tf关系

### 使用方法

```c++
// 生成一个 tf::TransformListener 用于查询tf信息
tf::TransformListener listener;
// 生成一个 tf::StampedTransform 对象用于获取查询到的tf信息
tf::StampedTransform ts;

// 使用try-catch方式查询
try
{
    // 查询turtle2坐标系在turtle1坐标系中的位置和方向，存入ts， time(0)表示获取到的tf关系是最新更新的
    listener.lookupTransform("/turtle2", "/turtle1", ros::Time(0), ts);
}
catch(const std::exception& e)
{
    std::cerr << e.what() << '\n';
    ros::Duration(1.0).sleep();
}

// 通过getOrigin()方法来获取到vector3向量
// 通过 getX() / x() 两种方法来获取vector3中的平移信息
// 注意没有getW()方法
double x = ts.getOrigin().getX();
double x = ts.getOrigin().x();

// 通过getRotation()来获取到quaternion信息
// 通过 getX() / x() 两种方法来获取quaternion中的平移信息
// 注意没有getW()方法
double angle = ts.getRotation().w();
```
