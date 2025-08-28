| Group |  Parameter | Type   | Units | Default Value | Required | Description                            | Notes                             |
|:-----:|:----------:|:------:|:-----:|:-------------:|:--------:|:--------------------------------------:|:---------------------------------:|
|       | node_name  | string | -     | ClockSubYarp  | No       | Set the name for ROS node              | must not start with a leading '/' |
|       | topic_name | string | -     | /clock        | No       | Specify the name of ROS2 clock topic   | must start with a leading '/'     |
|       | port_name  | string | -     | /clock        | No       | Specify the name of clock port to open | must start with a leading '/'     |
