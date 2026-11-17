"""
Markers of the Crospi Playground, controlled from a skill; published as RViz markers outside the playground.

In the playground the markers are defined in the Markers tab of the Skill mode (JSON). A skill can change them while
it runs: show/hide them, change a property (position, color, reference frame, ...), switch a property between a
fixed value and an eTaSL expression, add or remove markers. These changes are a layer on top of the definitions of
the tab: the tab is not modified, and the layer is cleared at every Run and Reset.

    from crospi_playground import playground_markers as mk

    mk.hide("target")
    mk.update("target", color="#ff0000", radius=0.03)
    mk.set_reference("target", "tool0")                  # follows the robot's tool0 link
    mk.set_position("target", [0.4, 0.0, 0.3])           # fixed value
    mk.set_position("target", mk.expr("origin(task_frame) + vector(0, 0, 0.1)"))  # eTaSL expression
    mk.add("goal", "sphere", position=[0.5, 0, 0.3], radius=0.02, color="#59a14f", opacity=0.5)
    mk.freeze("goal")                                    # links → their current values (e.g. drop a pin)
    mk.remove("goal")
    mk.reset("target")                                   # back to the definition of the tab

Markers are identified by their "name". Property names and values are those of the Markers tab (see its Form view
and the README, "Markers"). Problems (unknown marker, unknown property, ...) are logged as warnings; they never raise,
so the skill keeps running. For BeTFSM states (MarkerEdit, MarkerUpdater, WithMarkers), see betfsm_markers.

Outside the playground (the same skill run with crospi and ROS 2), the markers are published as RViz markers
(visualization_msgs/Marker on /visualization_marker, namespace = marker name), as far as RViz can show them (see
_rviz.py: no cones; eTaSL expressions use their default; warnings once per marker). There is no Markers tab there:
markers must be added by the skill (add) or loaded from a file saved from the tab (load). The ROS node is the BeTFSM
node of the skill, or the one given to init(node). Without rclpy, every function does nothing.
The package has no other dependencies: copy the folder crospi_playground next to the skill (or on the Python path).
"""
import json
import sys
import warnings

__all__ = ["AVAILABLE", "BACKEND", "PLAYGROUND", "init", "load", "expr", "update", "show", "hide", "set_position",
           "set_reference", "set_color", "add", "remove", "reset", "freeze", "get", "names"]


def _find_playground():
    """Bridge to the page (Markers tab of the Skill mode), or None outside the playground."""
    if sys.platform != "emscripten":
        return None
    try:
        from etasl_web import host
    except ImportError:
        return None
    js = getattr(host, "_js", None)
    if js is None or getattr(js, "markers", None) is None:
        return None
    return host


def _warn(text):
    if _host is not None:
        _host._log("warn", f"[markers] {text}")
    else:
        warnings.warn(f"[markers] {text}", stacklevel=4)


class _PlaygroundBackend:
    def op(self, op, **args):
        return json.loads(str(_host._js.markers.op(json.dumps({"op": op, **args}))))


_host = _find_playground()
_backend = None
if _host is not None:
    _backend = _PlaygroundBackend()
    BACKEND = "playground"
else:
    try:
        import rclpy  # noqa: F401
        from visualization_msgs.msg import Marker  # noqa: F401
        from crospi_playground._rviz import RVizBackend
        _backend = RVizBackend(_warn)
        BACKEND = "rviz"
    except ImportError:
        BACKEND = None

PLAYGROUND = BACKEND == "playground"
"""True in the Crospi Playground."""
AVAILABLE = _backend is not None
"""True when the markers are drawn: in the playground, or as RViz markers (rclpy available)."""

if BACKEND == "rviz":
    warnings.warn("crospi_playground.playground_markers: not running in the Crospi Playground, markers are published "
                  "as RViz markers on /visualization_marker (supported subset, see the warnings)", stacklevel=2)
elif BACKEND is None:
    warnings.warn("crospi_playground.playground_markers: not running in the Crospi Playground and no rclpy, "
                  "markers are ignored", stacklevel=2)


# ====================================================================== values
def expr(source, default=None):
    """
    Link to an eTaSL expression evaluated in the current task (with its top-level locals and globals in scope), e.g.
    expr("tcp") or expr("origin(task_frame) + vector(0, 0, 0.1)"). default: value used while the current task cannot
    evaluate it (visibility "always"), and outside the playground (RViz).
    """
    link = {"etasl": str(source)}
    if default is not None:
        link["default"] = _plain(default)
    return link


def _plain(value):
    """JSON value: tuples, numpy arrays/scalars, ... → lists and numbers."""
    if value is None or isinstance(value, (bool, str)):
        return value
    if isinstance(value, dict):
        return {str(k): _plain(v) for k, v in value.items()}
    if hasattr(value, "tolist"):  # numpy
        return _plain(value.tolist())
    if isinstance(value, (list, tuple)):
        return [_plain(v) for v in value]
    if isinstance(value, (int, float)):
        return value
    try:
        return float(value)
    except (TypeError, ValueError):
        return str(value)


def _color(color):
    """'#rrggbb', or (r, g, b) / (r, g, b, a) in 0..1 → ('#rrggbb', opacity or None)."""
    if isinstance(color, str):
        return color, None
    c = list(_plain(color))
    if len(c) not in (3, 4):
        _warn(f"color {color!r}: expected '#rrggbb' or (r, g, b[, a]) in 0..1")
        return None, None
    rgb = "#" + "".join(f"{max(0, min(255, round(float(x) * 255))):02x}" for x in c[:3])
    return rgb, (float(c[3]) if len(c) == 4 else None)


def _props(props, kw):
    out = dict(props or {})
    out.update(kw)
    if "color" in out and not isinstance(out["color"], str) and out["color"] is not None:
        rgb, alpha = _color(out["color"])
        out["color"] = rgb
        if alpha is not None and "opacity" not in out:
            out["opacity"] = alpha
    return {k: _plain(v) for k, v in out.items()}


# ====================================================================== bridge
def _call(op, **args):
    if _backend is None:
        return None
    try:
        reply = _backend.op(op, **args)
    except Exception as e:  # noqa: BLE001 (never stop the skill)
        _warn(f"{op}: {type(e).__name__}: {e}")
        return None
    for w in reply.get("warnings", []):
        _warn(w)
    return reply.get("result")


# ====================================================================== API
def init(node=None, base_frame=None, urdf=None):
    """
    Outside the playground (RViz): node = ROS 2 node used to publish (default: the BeTFSM node of the skill);
    base_frame = frame of "robot_base" (the robot's base link), or urdf = URDF (text or file path) whose root link is
    used; default: the root link of the URDF published on /robot_description. Does nothing in the playground.
    """
    if BACKEND == "rviz":
        try:
            _backend.init(node, base_frame, urdf)
        except Exception as e:  # noqa: BLE001
            _warn(f"init: {type(e).__name__}: {e}")


def load(path):
    """
    Outside the playground: loads marker definitions saved from the Markers tab ({"markers": [...]}, e.g. the
    <skill>.markers.json downloaded by Save). In the playground the tab is used and this does nothing.
    """
    if BACKEND == "rviz":
        _call("load", path=str(path))


def update(name, props=None, **kw):
    """
    Changes properties of the marker(s) named `name`, e.g. update("target", color="#ff0000", radius=0.03,
    position=expr("tcp")). props: dict of properties (also for names that are not valid Python identifiers). A value
    None removes the property (default of the marker type). color: '#rrggbb' or (r, g, b[, a]) in 0..1.
    """
    _call("update", name=name, props=_props(props, kw))


def show(name):
    """Draws the marker (property "enabled": true)."""
    _call("update", name=name, props={"enabled": True})


def hide(name):
    """Does not draw the marker (property "enabled": false)."""
    _call("update", name=name, props={"enabled": False})


def set_position(name, position):
    """Position: [x, y, z] (fixed value, in the marker's reference frame) or expr("...") (eTaSL expression)."""
    _call("update", name=name, props={"position": _plain(position)})


def set_reference(name, reference):
    """
    Frame of the marker's coordinates: "robot_base" (eTaSL world), "world" (simulation) or a body of the scene (a link
    of the robot's URDF, e.g. "tool0", or a body of the environment); the marker then follows that body.
    """
    _call("update", name=name, props={"reference": str(reference)})


def set_color(name, color, opacity=None):
    """color: '#rrggbb' or (r, g, b[, a]) in 0..1; opacity: 0..1."""
    p = {"color": color}
    if opacity is not None:
        p["opacity"] = opacity
    _call("update", name=name, props=_props(p, {}))


def add(name, type, props=None, **kw):  # noqa: A002 (as the Markers tab)
    """
    Adds or edits a marker: type = sphere, cylinder, box, plane, cone, point, frame, arrow, line, trail, text or path;
    properties as in the Markers tab, e.g. add("goal", "sphere", position=[0.5, 0, 0.3], radius=0.02, color="#59a14f").
    A marker with this name and type (e.g. of the Markers tab) is edited: only the given properties change (None
    removes one: its default). Otherwise the marker is created with the given properties, the others take their
    defaults (a marker with this name of another type is replaced).
    """
    _call("add", name=name, type=str(type), props=_props(props, kw))


def remove(name):
    """Removes the marker(s) named `name` (also one of the Markers tab, until the next Run or Reset)."""
    _call("remove", name=name)


def reset(name=None):
    """Drops the changes of the skill to the marker named `name` (None: to all markers)."""
    _call("reset", name=name)


def freeze(name):
    """
    Replaces the eTaSL expressions of the marker by their current values: the marker stays where it is now, e.g. a
    contact point dropped at the end of a guarded motion. (Outside the playground the values are already fixed.)
    """
    _call("freeze", name=name)


def get(name):
    """Current definition of the marker named `name` (tab + changes of the skill) as a dict, or None."""
    return _call("get", name=name)


def names():
    """Names of the current markers (tab + changes of the skill); [] when nothing is drawn."""
    return _call("names") or []
