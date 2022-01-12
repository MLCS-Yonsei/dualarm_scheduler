# dualarm_scheduler
Task scheduling package for dual-arm mobile manipulator

## Parameters
```
nodes: [name_of_the_node_1, name_of_the_node2,...]

tasks: [
  [node_for_task_1, additional_command_1, additional_command_2,...],
  [node_for_task_2, ],
  ...
]
```

## Subscribed & Published Topics
```
name_of_the node/task_flag (std_msgs/Int8)
```
- Message Definition:
```
# ERROR=-1
# INACTIVE=0
# ACTIVE=1
# SUCCESS=2
int8 data
```
