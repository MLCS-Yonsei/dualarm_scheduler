#!/usr/bin/env python

import rospy
from std_msgs.msg import Int8
import time
from subprocess import call

ERROR = -1
INACTIVE = 0
ACTIVE = 1
SUCCESS = 2



class NodeContainer:

    def __init__(self, node):
        self.flag = INACTIVE
        self.pub = rospy.Publisher(
            node + "/task_flag/from",
            Int8,
            queue_size=10
        )
        def flagCallback(msg):
            self.flag = msg.data
            rospy.set_param(node + "/task_flag", msg.data)
        self.sub = rospy.Subscriber(
            node + "/task_flag/to",
            Int8,
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
            rospy.set_param(node + "/task_flag", INACTIVE)
        self.tasks = rospy.get_param("/dualarm_scheduler/tasks")
        self.scheduler_flag = INACTIVE
        def Callback(msg):
            if msg.data==ERROR:
                self.scheduler_flag = ERROR
            elif msg.data==ACTIVE:
                self.scheduler_flag = ACTIVE
        self.sub = rospy.Subscriber(
            "/dualarm_scheduler/flag",
            Int8,
            Callback
        )
        self.isInitialized = True


    def spin(self):
        rate = rospy.Rate(self.loop_rate)
        while not rospy.is_shutdown():
            if self.scheduler_flag==ERROR:
                self.terminate()
                break
            elif self.scheduler_flag==ACTIVE:
                task_lock = False
                for node in self.nodes.keys():
                    if self.nodes[node].flag==ERROR:
                        self.terminate()
                        break
                    elif self.nodes[node].flag==ACTIVE:
                        task_lock = True
                        break
                    else:
                        self.nodes[node].flag = INACTIVE
                        rospy.set_param(node + "/task_flag", INACTIVE)
                if not task_lock:
                    if len(self.tasks) > 0:
                        args = self.tasks.pop(0)
                        print("current task:", args)
                        self.nodes[args[0]].flag = ACTIVE
                        self.nodes[args[0]].pub.publish(ACTIVE)
                        if len(args) > 1:
                            for arg in args[1:]:
                                call([arg], shell=True)
                    else:
                        break
            rate.sleep()


    def cleanup(self):
        for node in self.nodes.keys():
            self.nodes[node].pub.publish(INACTIVE)


    def terminate(self):
        for node in self.nodes.keys():
            self.nodes[node].pub.publish(ERROR)



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
