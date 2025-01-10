# SLAM-tool-kit

Two simple ROS2 Humble nodes you can use to:
- publish Kitti stereo dataset on 2 topics (/cam0/image_raw and /cam1/image_raw) and Kitti poses on another node (/ground_truth/odom)
- evaluate SLAM by comparing calculated poses and true poses (subscriptions: calculated poses - /vo_pose , true poses - /ground_truth/odom); publishes deviations on an ongoing basis (topic: /evaluation) and sums them up with additional metrics (rmse, me, sd) in a file (examples of the files in evaluations)

Final version tested and used on:
[https://github.com/ov2slam/ov2slam]
