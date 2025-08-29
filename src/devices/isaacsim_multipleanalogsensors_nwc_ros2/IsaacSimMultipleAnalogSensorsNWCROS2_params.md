| Group |  Parameter       | Type           | Units    | Default Value        | Required  | Description                                       | Notes                                      |
|:-----:|:----------------:|:--------------:|:--------:|:--------------------:|:---------:|:-------------------------------------------------:|:------------------------------------------:|
|       | node_name        | string         | -        | MASBridgeSubscriber  | No        | Set the name for ROS node                         | must not start with a leading '/'          |
|       | ft_topic_names   | vector<string> | -        |   -                  | Yes       | Specify the name of the FT topics to connect      | Each element must start with a leading '/' |
|       | imu_topic_names  | vector<string> | -        |   -                  | Yes       | Specify the name of the IMU topics to connect     | Each element must start with a leading '/' |
