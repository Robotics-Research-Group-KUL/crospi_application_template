"""
Debugging states of the Crospi Playground for BeTFSM skills: use them in a skill's tree like any other BeTFSM state.

    from crospi_playground.betfsm_debug import PlaygroundBreakpoint

    Sequence("demo", [
        CrospiTask("approach", ...),
        PlaygroundBreakpoint("check approach", message="Check the pose of the tool, then Resume"),  # breakpoint
        PlaygroundBreakpoint("if large force", when=lambda bb: bb["force"] > 10.0),                 # conditional breakpoint
        CrospiTask("insert", ...),
    ])

PlaygroundBreakpoint is a breakpoint: when it is reached, the playground pauses (as the Pause button), so that you can read
values (plots, HUD, transform assistant) or move the robot (Joints panel, transform assistant) before the skill
continues. Resume (or Step) continues: the state then succeeds. Everything runs in simulated time, so the skill,
eTaSL, the physics and the timers freeze together and continue exactly where they stopped.

If an eTaSL task is active when the breakpoint is reached (e.g. the state is in a Concurrent with a CrospiTask), the
whole simulation is frozen as well: reading values is safe, but a robot moved while paused is pulled back by the task
when it resumes (its constraints see the jump); the console warns about it.

"skip breakpoints" (Skill mode toolbar of the playground) runs the skill without stopping at them.

Outside the playground (the same skill run with crospi and ROS 2), PlaygroundBreakpoint does nothing: it logs a line and
succeeds immediately, so that a breakpoint left in a skill never stalls a real robot.
The package has no other dependencies: copy the folder crospi_playground next to the skill (or on the Python path).
"""
import sys

from betfsm import SUCCEED, TICKING, Generator, get_logger

__all__ = ["PlaygroundBreakpoint"]


def _find_playground():
    """Bridge to the page (pause of the playground), or None outside the playground."""
    if sys.platform != "emscripten":
        return None
    try:
        from etasl_web import host
    except ImportError:
        return None
    js = getattr(host, "_js", None)
    if js is None or getattr(js, "breakpoint", None) is None:
        return None
    return js


class PlaygroundBreakpoint(Generator):
    """
    Breakpoint: pauses the playground when reached; succeeds when the playground is resumed (Resume or Step).

        PlaygroundBreakpoint("check grasp")
        PlaygroundBreakpoint("check grasp", message="Is the box between the fingers?")
        PlaygroundBreakpoint("large force", when=lambda bb: bb["force"] > 10.0)   # only when the condition holds

    Parameters:
        name: name of the state (shown in the console and in the BeTFSM dashboard)
        message: text shown when it pauses (optional)
        when: callable f(blackboard) -> bool: pause only when it returns True (optional; default: always)

    Outcome: SUCCEED. Outside the playground: logs and succeeds immediately (no pause).
    """

    def __init__(self, name: str, message: str = "", when=None):
        super().__init__(name, [SUCCEED])
        self.message = message
        self.when = when

    def co_execute(self, blackboard):
        if self.when is not None and not self.when(blackboard):
            yield SUCCEED
            return
        js = _find_playground()
        if js is None:
            get_logger().info(f"PlaygroundBreakpoint '{self.name}': not in the Crospi Playground, continuing"
                              + (f" ({self.message})" if self.message else ""))
            yield SUCCEED
            return
        if str(js.breakpoint(self.name, self.message or "")) == "paused":
            # the playground does not tick the skill while paused: the next tick comes with Resume (or Step)
            yield TICKING
        yield SUCCEED
