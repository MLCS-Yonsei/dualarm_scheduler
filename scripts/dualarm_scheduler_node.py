#!/usr/bin/env python

import csv
import rospy
from std_msgs.msg import Bool
from geometry_msgs.msg import Twist
from geometry_msgs.msg import PoseStamped
import time



class NodeContainer:

    def __init__(self, node):
        self.flag = False
        self.sub = None
        self.pub = rospy.Publisher(
            node + "/task_flag",
            Bool,
            queue_size=10
        )
        def flagCallback(msg):
            self.flag = msg.data
            print(node, self.flag)
        self.sub = rospy.Subscriber(
            node + "/task_flag",
            Bool,
            flagCallback
        )



class SchedulerNode:

    def __init__(self):

        self.isInitialized = False
        rospy.init_node('dualarm_scheduler')
        time.sleep(1)
        rospy.on_shutdown(self.cleanup)

        self.loop_rate = rospy.get_param("/dualarm_scheduler/loop_rate")
        self.nodes = {}
        for node in rospy.get_param("/dualarm_scheduler/nodes"):
            self.nodes[node] = NodeContainer(node)
        self.tasks = rospy.get_param("/dualarm_scheduler/tasks")
        print(self.tasks)
        self.isInitialized = True


    def spin(self):
        rate = rospy.Rate(self.loop_rate)
        while not rospy.is_shutdown():
            task_lock = False
            for node in self.nodes.keys():
                if self.nodes[node].flag:
                    task_lock = True
                    break
            if not task_lock:
                if len(self.tasks) > 0:
                    args = self.tasks.pop(0)
                    print(args)
                    self.nodes[args[0]].flag = True
                    self.nodes[args[0]].pub.publish(True)
                    if len(args) > 1:
                        exec 'from ' + args[2].split('/')[0] + '.msg import ' + args[2].split('/')[-1]
                        task_pub = rospy.Publisher(
                            args[1],
                            eval(args[2].split('/')[-1]),
                            queue_size=10
                        )
                        time.sleep(1)
                        task_pub.publish(args[3])
                        del task_pub
                else:
                    break
            rate.sleep()


    def cleanup(self):
        for node in self.nodes.keys():
            self.nodes[node].pub.publish(False)



if __name__ == '__main__':

    try:
        node = SchedulerNode()
        tic = time.time()
        while not node.isInitialized:
            toc = time()
            if toc - tic > 10:
                rospy.loginfo("dualarm_scheduler node initialization timeout (10s).")
                break
        if node.isInitialized:
            time.sleep(1)
            node.spin()
    except rospy.ROSInterruptException:
        rospy.loginfo("dualarm_scheduler node terminated.")
