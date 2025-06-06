import rospy
from sensor_msgs.msg import PointCloud2
from message_filters import ApproximateTimeSynchronizer, Subscriber
from std_msgs.msg import Header

def callback(lidar1_data, lidar2_data):
    # Process synchronized data here
    # lidar1_data and lidar2_data are synchronized PointCloud2 messages

    # Example: Publishing combined data to a new topic
    combined_pc = combine_pointclouds(lidar1_data, lidar2_data)  # Your function to combine data
    combined_pub.publish(combined_pc)

def combine_pointclouds(pc1, pc2):
    '''
    This function should combines data from pc1 and pc2 into a single PointCloud2 message
    '''
    combined_points = pc1.data + pc2.data  # points are stored in 'data' field
    combined_header = Header()
    combined_header.stamp = rospy.Time.now()
    combined_header.frame_id = msg.header.frame_id  # Assign an appropriate frame ID

    combined_pc = PointCloud2()
    combined_pc.header = combined_header
    combined_pc.data = combined_points  # Assign the combined points to the data field

    return combined_pc

rospy.init_node('lidar_sync_node')
lidar1_sub = Subscriber('/ns1/velodyne_points', PointCloud2)
lidar2_sub = Subscriber('/ns2/velodyne_points', PointCloud2)
sync = ApproximateTimeSynchronizer([lidar1_sub, lidar2_sub], queue_size=10, slop=0.1)
sync.registerCallback(callback)

# Publisher for the combined synchronized data
combined_pub = rospy.Publisher('/combined_lidar_topic', PointCloud2, queue_size=10)

rospy.spin()

