
#  Copyright (c) 2025 KU Leuven, Belgium
#
#  Author: Santiago Iregui
#  email: <santiago.iregui@kuleuven.be>
#
#  GNU Lesser General Public License Usage
#  Alternatively, this file may be used under the terms of the GNU Lesser
#  General Public License version 3 as published by the Free Software
#  Foundation and appearing in the file LICENSE.LGPLv3 included in the
#  packaging of this file. Please review the following information to
#  ensure the GNU Lesser General Public License version 3 requirements
#  will be met: https://www.gnu.org/licenses/lgpl.html.
# 
#  This program is distributed in the hope that it will be useful,
#  but WITHOUT ANY WARRANTY; without even the implied warranty of
#  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
#  GNU Lesser General Public License for more details.

import sys

from crospi_interfaces.srv import TaskSpecificationFile
from crospi_interfaces.srv import TaskSpecificationString
from crospi_py import etasl_params

import rclpy
from rclpy.node import Node


class MinimalEtaslClient(Node):

# /crospi_node/readRobotSpecification
    def __init__(self):
        super().__init__('minimal_etasl_client')
        self.readTaskSpecificationFileClient = self.create_client(TaskSpecificationFile, '/crospi_node/readTaskSpecificationFile')
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
    etasl_params.load_task_list("$[crospi_application_template]/coordination/betfsm/task_configuration/up_and_down_exampletask_specifications",blackboard) #Loads JSON into the blackboard dictionary


    minimal_client = MinimalEtaslClient()
    response = minimal_client.readTaskSpecificationFile()
    # minimal_client.get_logger().info(
    #     'Result of add_two_ints: for %d + %d = %d' %
    #     (int(sys.argv[1]), int(sys.argv[2]), response.sum))

    minimal_client.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()