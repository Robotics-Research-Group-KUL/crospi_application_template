import sys

from etasl_interfaces.srv import TaskSpecificationFile
from etasl_interfaces.srv import TaskSpecificationString
from etasl_ros2_py import etasl_params

import rclpy
from rclpy.node import Node


class MinimalEtaslClient(Node):

# /etasl_node/readRobotSpecification
    def __init__(self):
        super().__init__('minimal_etasl_client')
        self.readTaskSpecificationFileClient = self.create_client(TaskSpecificationFile, '/etasl_node/readTaskSpecificationFile')
        while not self.readTaskSpecificationFileClient.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')



    def readTaskSpecificationFile(self, file_path):
        req = TaskSpecificationFile.Request()
        req.a = a
        req.b = b
        self.future = self.readTaskSpecificationFileClient.call_async(req)
        rclpy.spin_until_future_complete(self, self.future)
        return self.future.result()


def main(args=None):
    rclpy.init(args=args)

    blackboard = {} # Empty dictionary to simplify example
    task_name = "movingHome"
    etasl_params.load_task_list("$[etasl_ros2_application_template]/coordination/betfsm/task_configuration/up_and_down_exampletask_specifications",blackboard) #Loads JSON into the blackboard dictionary


    minimal_client = MinimalEtaslClient()
    response = minimal_client.readTaskSpecificationFile()
    # minimal_client.get_logger().info(
    #     'Result of add_two_ints: for %d + %d = %d' %
    #     (int(sys.argv[1]), int(sys.argv[2]), response.sum))

    minimal_client.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()