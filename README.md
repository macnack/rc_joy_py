# rc_joy
<!-- Required -->
<!-- Package description -->

## Installation
<!-- Required -->
<!-- Things to consider:
    - How to build package? 
    - Are there any other 3rd party dependencies required? -->

```bash
rosdep install --from-paths src --ignore-src -y
colcon build --symlink-install --packages-up-to rc_joy
```

## Usage
<!-- Required -->
<!-- Things to consider:
    - Launching package. 
    - Exposed API (example service/action call. -->

```bash
ros2 launch rc_joy rc_joy.launch.py
```

## API
<!-- Required -->
<!-- Things to consider:
    - How do you use the package / API? -->

### Input

| Name         | Type                  | Description  |
| ------------ | --------------------- | ------------ |
| `topic_name` | std_msgs::msg::String | Sample desc. |

### Output

| Name         | Type                  | Description  |
| ------------ | --------------------- | ------------ |
| `topic_name` | std_msgs::msg::String | Sample desc. |

### Services and Actions

| Name           | Type                   | Description  |
| -------------- | ---------------------- | ------------ |
| `service_name` | std_srvs::srv::Trigger | Sample desc. |

### Parameters

| Name         | Type | Description  |
| ------------ | ---- | ------------ |
| `param_name` | int  | Sample desc. |


## References / External links
<!-- Optional -->
