"""
BeTFSM states for the simulated parallel gripper of the Crospi Playground ("gripper" of the MuJoCo simulator in the
setup), over the ROS 2 topics of the simulator (an interface of the playground, not of a real gripper):

    command: std_msgs/Float64MultiArray  data = [opening [m], force [N] (optional)]     (default /gripper/command)
    state:   sensor_msgs/JointState      the finger joints (position, velocity, effort)  (default /gripper/joint_states)

The opening is the sum of the positions of the finger joints (0: closed).

Only for the simulation: real grippers have their own drivers and interfaces (usually ROS 2 actions, e.g.
control_msgs/action/GripperCommand), for which states have to be written for the specific gripper.

    Sequence("grasp", [
        GripperOpen("open"),
        ...
        GripperClose("close", force=20.0),     # SUCCEED: an object is held; CANCEL: the fingers closed completely
    ])

Outcomes: SUCCEED; CANCEL (GripperClose: nothing between the fingers); TIMEOUT (the gripper did not stop in time).
"""
import time

from betfsm import CANCEL, SUCCEED, TICKING, TIMEOUT, Generator, get_logger
from betfsm_ros import BeTFSMNode
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray

COMMAND_TOPIC = "/gripper/command"
STATE_TOPIC = "/gripper/joint_states"

_caches = {}  # per node: state topic → {"msg": latest JointState, "count": number of messages}; command topic → publisher


def _cache():
    """Subscriptions and publishers of the node of the skill (a new node, e.g. another run: new ones)."""
    node = BeTFSMNode.get_instance()
    return _caches.setdefault(id(node), {})


def _state(topic):
    topics = _cache()
    s = topics.get(topic)
    if s is None:
        s = topics[topic] = {"msg": None, "count": 0}

        def cb(msg, s=s):
            s["msg"] = msg
            s["count"] += 1
        BeTFSMNode.get_instance().create_subscription(JointState, topic, cb, 10)
    return s


def _publisher(topic):
    topics = _cache()
    key = ("pub", topic)
    if key not in topics:
        topics[key] = BeTFSMNode.get_instance().create_publisher(Float64MultiArray, topic, 10)
    return topics[key]


class GripperMove(Generator):
    """
    Moves the fingers to an opening [m] (clamped by the gripper to its range) with a maximal force [N] per finger
    (0: the default of the gripper) and waits until they stop: SUCCEED, or TIMEOUT after `timeout` seconds. The final
    opening is written into the blackboard: blackboard["gripper"]["opening"].

    opening and force can be callables f(blackboard).
    """

    def __init__(self, name, opening, force=0.0, timeout=5.0, settle=0.2, outcomes=(SUCCEED, TIMEOUT),
                 command_topic=COMMAND_TOPIC, state_topic=STATE_TOPIC):
        super().__init__(name, list(outcomes))
        self.opening, self.force = opening, force
        self.timeout, self.settle = timeout, settle
        self.command_topic, self.state_topic = command_topic, state_topic

    def result(self, opening, target):
        return SUCCEED

    def co_execute(self, blackboard):
        target = self.opening(blackboard) if callable(self.opening) else self.opening
        force = self.force(blackboard) if callable(self.force) else self.force
        state = _state(self.state_topic)
        _publisher(self.command_topic).publish(Float64MultiArray(data=[float(target), float(force)]))
        start = time.monotonic()
        count0 = state["count"]
        still_since = None
        while True:
            now = time.monotonic()
            msg = state["msg"]
            if msg is not None and state["count"] >= count0 + 3:  # a few states after the command
                speed = abs(sum(msg.velocity)) if msg.velocity else 0.0
                if speed < 2e-3:
                    still_since = now if still_since is None else still_since
                    if now - still_since >= self.settle:
                        opening = float(sum(msg.position))
                        blackboard.setdefault("gripper", {})["opening"] = opening
                        outcome = self.result(opening, target)
                        get_logger().info(f"{self.name}: gripper stopped at an opening of {1000 * opening:.1f} mm ({outcome})")
                        yield outcome
                        return
                else:
                    still_since = None
            if now - start > self.timeout:
                get_logger().warning(f"{self.name}: the gripper did not stop within {self.timeout} s"
                                     f"{'' if msg is not None else f' (no state on {self.state_topic})'}")
                yield TIMEOUT
                return
            yield TICKING


class GripperOpen(GripperMove):
    """Opens the gripper (completely by default, or to `opening` [m])."""

    def __init__(self, name, opening=1.0, force=0.0, **kw):
        super().__init__(name, opening, force, **kw)


class GripperClose(GripperMove):
    """
    Closes the gripper with a maximal force [N] per finger: SUCCEED when the fingers stop on an object (opening larger
    than `min_opening` [m]), CANCEL when they close (nothing between them), TIMEOUT after the timeout.
    """

    def __init__(self, name, force=0.0, min_opening=0.003, **kw):
        super().__init__(name, 0.0, force, outcomes=(SUCCEED, CANCEL, TIMEOUT), **kw)
        self.min_opening = min_opening

    def result(self, opening, target):
        return SUCCEED if opening > self.min_opening else CANCEL
