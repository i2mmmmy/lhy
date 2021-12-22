# TIPS

# Topic
* 函数声明  
`Publisher advertise(const std::string& topic, uint32_t queue_size, bool latch = false)`
* 默认发布话题函数  
`ros::Publisher pub = handle.advertise<std_msgs::Empty>("my_topic", 1);`
* 加锁存后的发布函数  
`ros::Publisher pub = handle.advertise<std_msgs::Empty>("my_topic", 1, true);`  
	* 加锁存后，当一个新的订阅者订阅某个已经存在的话题时，能够获取该话题最后一次发布的消息
* 发布函数可以加入回调函数(在被订阅和取消订阅时调用)
	* 函数声明
	```c++
	  template <class M>
	  Publisher advertise(const std::string& topic, uint32_t queue_size,
		                        const SubscriberStatusCallback& connect_cb,
		                        const SubscriberStatusCallback& disconnect_cb = SubscriberStatusCallback(),
		                        const VoidConstPtr& tracked_object = VoidConstPtr(),
		                        bool latch = false)
	```
	* 实例：
	当有新的节点订阅到此话题时会触发回调， 注意：默认第一个是订阅的回调，第二个是取消订阅的回调
	```c++
	...
	void ConnectCB(const ros::SingleSubscriberPublisher& p_pub)
	{
		std::cout << "Connect " << p_pub.getTopic() << std::endl;
	}
	...
    ros::Publisher p_pub = nh.advertise<std_msgs::UInt16>("auto_pub", 1, (ros::SubscriberStatusCallback)ConnectCB, (ros::SubscriberStatusCallback)disConnectCB);
	...
	```
# Service


# 获取节点名

getPrivateNodeHandle();




