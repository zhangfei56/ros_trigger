import rospy
from std_msgs.msg import String
from rosbag import Bag
from nav_msgs.msg import Odometry


def write_bag_file(bag_filename):
    
    # # 创建一个新的 ROS bag 文件
    bag =  Bag(bag_filename, 'w')
    # 创建一个 ROS 节点
    rospy.init_node('bag_writer', anonymous=True)
    
    def callback(data):
        bag.write('/odom', data)
    rospy.Subscriber('/odom', Odometry, callback)
    
    
        
    rospy.spin()
    # bag.reindex()
    # bag.flush()
    bag.close()
    
    # with Bag(bag_filename, 'w') as bag:
    #     rospy.init_node('bag_writer', anonymous=True)
    
    #     def callback(data):
    #         bag.write('/odom', data)
    #     rospy.Subscriber('/odom', Odometry, callback)
        
    
    #     rospy.spin()


if __name__ == '__main__':
    try:
        write_bag_file('my_data.bag')
    except rospy.ROSInterruptException:
        print("error")
