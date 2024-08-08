# Script created by modifying the MonitorState class found in https://github.com/uleroboticsgroup/yasmin/blob/main/yasmin_ros/yasmin_ros/monitor_state.py
# Santiago Iregui, 2024
# KU Leuven, Robotics Research Group (https://www.mech.kuleuven.be/en/pma/research/robotics)

import time
from typing import List, Callable, Union, Type


from rclpy.node import Node
# from rclpy.subscription import Subscription

from yasmin import State
from yasmin import Blackboard
from yasmin_ros.yasmin_node import YasminNode
from yasmin_ros.basic_outcomes import CANCEL, TIMEOUT

from rclpy.qos import QoSHistoryPolicy, QoSProfile, QoSReliabilityPolicy, QoSDurabilityPolicy
from std_msgs.msg import String
from colorama import Fore, Back, Style



class EventState(State):

    def __init__(
        self,
        topic_name: str,
        outcomes: List[str],
        node: Node = None,
        entry_handler: Callable = None,
        monitor_handler: Callable=None,
        exit_handler: Callable = None,
        msg_queue: int = 30,
        timeout: int = None, # timeout to wait for msgs in seconds. if not None, CANCEL outcome is added and returned after timeout
        state_name: str = None
    ) -> None:

        if not timeout is None:
            outcomes = [CANCEL, TIMEOUT] + outcomes
        super().__init__(outcomes)

        if monitor_handler is  None:
            self.monitor_handler = self.default_monitor_handler
        else:
            self.monitor_handler = monitor_handler
        self.entry_handler = entry_handler
        self.exit_handler = exit_handler
        self.msg_list = []
        self.msg_queue = msg_queue
        self.timeout = timeout
        self.time_to_wait = 0.001
        self.monitoring = False
        self.outcomes = outcomes
        self.state_name = state_name

        qos_profile = QoSProfile(
            history=QoSHistoryPolicy.KEEP_LAST, #Keeps the last msgs received in case buffer is fulll
            depth=msg_queue, #Buffer size
            reliability=QoSReliabilityPolicy.RELIABLE, #Uses TCP for reliability instead of UDP
            durability=QoSDurabilityPolicy.VOLATILE #Volatile, may not use first msgs if subscribed late (will not happen in this context)
        )

        if node is None:
            self.node = YasminNode.get_instance()
        else:
            self.node = node

        self._sub = self.node.create_subscription(
            String, topic_name, self.__callback, qos_profile)

    def __callback(self, msg) -> None:

        if self.monitoring:
            self.msg_list.append(msg)

            if len(self.msg_list) >= self.msg_queue:
                self.msg_list.pop(0)

    def execute(self, blackboard: Blackboard) -> str:
        if not self.state_name is None:
            print(Style.BRIGHT + Fore.GREEN + 'ENTERING STATE ' + self.state_name + Style.RESET_ALL)

        elapsed_time = 0
        self.msg_list = []
        self.monitoring = True
        valid_transition = False
        if not self.entry_handler is None:
            self.entry_handler(blackboard)


        while not valid_transition:
            time.sleep(self.time_to_wait)

            if not self.timeout is None:

                if elapsed_time >= self.timeout:
                    self.monitoring = False
                    return TIMEOUT

                elapsed_time += self.time_to_wait

            if self.msg_list:
                outcome = self.monitor_handler(blackboard, self.msg_list[0])
                # if outcome is None:
                #     # self.node.get_logger().warn("Transition undeclared or declared but unhandled.")
                #     self.msg_list.pop(0)
                if outcome is not None and outcome in self.outcomes:
                    valid_transition = True
                    if not self.exit_handler is None:
                        self.exit_handler(blackboard)
                    if not self.state_name is None:
                        print(Style.BRIGHT + Fore.RED + 'EXITING STATE ' + self.state_name + Style.RESET_ALL)
                    break
                else:
                    self.msg_list.pop(0)

        self.monitoring = False
        return outcome

    def default_monitor_handler(self, blackboard: Blackboard, msg: String) -> str:
        self.node.get_logger().info("Event received: {}".format(msg.data))
        return msg.data
