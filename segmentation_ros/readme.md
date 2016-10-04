This node is a wrapper for the segmentationm library.

The node can be used in three ways:

1) using pubish subscribe nethods for continuous processing of point clouds. This behaviour can be enabled/disabled by a service call. Default value is disabled

2) using service calls. The input param for the service call is a point cloud, the returned result contains the segmented point cloud. The service call is a blocking call.

3) using the action server. The action call used an empty goal. A fresh point cloud is taken from the topic and processed exactly once in a non blocking manner. There is no feedback given because this would be overkill in this case. The result from the action server contains the segmented point cloud as a payload.




