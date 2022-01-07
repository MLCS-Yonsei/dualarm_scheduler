# dualarm_scheduler
Task scheduling package for dual-arm mobile manipulator

## Parameters
```
nodes: [name_of_the_node_1, name_of_the_node2,...]

tasks: [
  [node_for_task_1, additional_msg_topic, additional_msg_type, additional_msg_value],
  [node_for_task_2, ],
  ...
]
```

## Subscribed Topics
```
name_of_the node/task_flag (std_msgs/Bool)
```

## Published Topics
```
name_of_the node/task_flag (std_msgs/Bool)
additional_msg_topic (additional_msg_type)
```
