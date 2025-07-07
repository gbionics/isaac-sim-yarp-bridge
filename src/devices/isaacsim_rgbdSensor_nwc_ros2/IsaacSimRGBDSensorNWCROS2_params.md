| Group |  Parameter       | Type      | Units    | Default Value        | Required  | Description                                          | Notes                             |
|:-----:|:----------------:|:---------:|:--------:|:--------------------:|:---------:|:----------------------------------------------------:|:---------------------------------:|
|       | node_name        | string    | -        | RGBDBridgeSubscriber | No        | Set the name for ROS node                            | must not start with a leading '/' |
|       | rgb_topic_name   | string    | -        |   -                  | Yes       | Specify the name of the RGB topic to connect         | must start with a leading '/'     |
|       | depth_topic_name | string    | -        |   -                  | Yes       | Specify the name of the depth image topic to connect | must start with a leading '/'     |
