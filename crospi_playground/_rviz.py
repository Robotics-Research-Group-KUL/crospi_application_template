"""
RViz fallback of playground_markers (outside the Crospi Playground): the marker definitions are kept here (markers
added by the skill, or loaded with load()) and published as visualization_msgs/Marker messages on
/visualization_marker (namespace = name of the marker), on every change and once per second (for RViz instances
started later).

Translation (playground → RViz): sphere → SPHERE, point → SPHERE, box → CUBE, cylinder → CYLINDER, plane → thin CUBE,
arrow → ARROW, line → LINE_STRIP, frame → 3 ARROWs, text → TEXT_VIEW_FACING, path → LINE_STRIP (curve sampled as in
the playground) + SPHERE_LIST (show_points), trail → LINE_STRIP of the positions given by the skill. Not supported:
cone. eTaSL links cannot be evaluated outside the playground: their "default" is used, else the property is left out.
"visibility" is ignored. Each incompatibility is warned once per marker.

reference: "robot_base" → the root link of the robot's URDF (init(urdf=...), else the /robot_description topic,
else init(base_frame=...); "base_link" until known), "world" → "world", a body → that link (published by
robot_state_publisher in TF).
"""
import json
import math
import time
import xml.etree.ElementTree as ET

# defaults of the properties (as MARKER_TYPES in web/js/markers.js)
TYPES = {
    "sphere": {"position": [0.5, 0, 0.5], "radius": 0.05},
    "cylinder": {"position": [0.5, 0, 0.5], "axis": [0, 0, 1], "radius": 0.05, "length": 0.3},
    "box": {"position": [0.5, 0, 0.5], "orientation": [0, 0, 0], "size": [0.1, 0.1, 0.1]},
    "plane": {"position": [0.5, 0, 0.5], "normal": [0, 0, 1], "size": [0.4, 0.4]},
    "cone": {"position": [0.5, 0, 0.5], "axis": [0, 0, 1], "angle": 0.3, "length": 0.15},
    "point": {"position": [0.5, 0, 0.5], "size": 0.02},
    "frame": {"position": [0.5, 0, 0.5], "orientation": [0, 0, 0], "size": 0.1},
    "arrow": {"position": [0.5, 0, 0.5], "vector": [0, 0, 0.1], "scale": 1},
    "line": {"position": [0.5, 0, 0.5], "end": [0.5, 0, 0.8]},
    "trail": {"position": [0.5, 0, 0.5], "duration": 10},
    "text": {"position": [0.5, 0, 0.5], "text": "label", "size": 0.04},
    "path": {"points": [[0.4, -0.3, 0.3], [0.5, -0.1, 0.45], [0.4, 0.1, 0.3], [0.5, 0.3, 0.45]],
             "interpolation": "polyline", "degree": 3, "closed": False, "width": 0.004, "samples": 100,
             "show_points": False},
}
COMMON = ("name", "enabled", "visibility", "reference", "color", "opacity")


def type_of(m):
    return next((t for t in TYPES if m.get(f"is-{t}") is True), None)


def is_link(v):
    return isinstance(v, dict) and isinstance(v.get("etasl"), str)


# ====================================================================== curves (as pathPoints() in markers.js)
def _lerp(a, b, t):
    return [a[i] + (b[i] - a[i]) * t for i in range(3)]


def _catmull_rom(P, closed, samples):
    """Centripetal Catmull-Rom curve through P (as THREE.CatmullRomCurve3(P, closed, 'centripetal').getPoints)."""
    n = len(P)

    def point(t):
        p = (n - (0 if closed else 1)) * t
        i = math.floor(p)
        w = p - i
        if closed:
            i += 0 if i > 0 else (math.floor(abs(i) / n) + 1) * n
        elif w == 0 and i == n - 1:
            i, w = n - 2, 1.0
        p0 = P[(i - 1) % n] if closed or i > 0 else [2 * P[0][k] - P[1][k] for k in range(3)]
        p1, p2 = P[i % n], P[(i + 1) % n]
        p3 = P[(i + 2) % n] if closed or i + 2 < n else [2 * P[n - 1][k] - P[n - 2][k] for k in range(3)]
        d2 = lambda a, b: sum((a[k] - b[k]) ** 2 for k in range(3))  # noqa: E731
        dt0, dt1, dt2 = d2(p0, p1) ** 0.25, d2(p1, p2) ** 0.25, d2(p2, p3) ** 0.25
        if dt1 < 1e-4:
            dt1 = 1.0
        if dt0 < 1e-4:
            dt0 = dt1
        if dt2 < 1e-4:
            dt2 = dt1
        out = []
        for k in range(3):
            x0, x1, x2, x3 = p0[k], p1[k], p2[k], p3[k]
            t1 = ((x1 - x0) / dt0 - (x2 - x0) / (dt0 + dt1) + (x2 - x1) / dt1) * dt1
            t2 = ((x2 - x1) / dt1 - (x3 - x1) / (dt1 + dt2) + (x3 - x2) / dt2) * dt1
            c2 = -3 * x1 + 3 * x2 - 2 * t1 - t2
            c3 = 2 * x1 - 2 * x2 + t1 + t2
            out.append(x1 + t1 * w + c2 * w * w + c3 * w * w * w)
        return out

    return [point(j / samples) for j in range(samples + 1)]


def _bspline(P, k, closed, samples):
    k = max(1, min(int(k), len(P) - 1))
    C = P + P[:k] if closed else P
    n = len(C)
    if closed:
        knots = list(range(n + k + 1))
        t0, t1 = knots[k], knots[n]
    else:
        inner = n - k - 1
        knots = [0.0] * (k + 1) + [(i + 1) / (inner + 1) for i in range(inner)] + [1.0] * (k + 1)
        t0, t1 = 0.0, 1.0
    out = []
    for j in range(samples + 1):
        t = t0 + (t1 - t0) * (j / samples)
        s = k
        while s < n - 1 and t >= knots[s + 1]:
            s += 1
        d = [list(C[s - k + i]) for i in range(k + 1)]
        for r in range(1, k + 1):
            for i in range(k, r - 1, -1):
                a = s - k + i
                den = knots[a + k + 1 - r] - knots[a]
                d[i] = _lerp(d[i - 1], d[i], (t - knots[a]) / den if den > 0 else 0)
        out.append(d[k])
    return out


def _bezier(P, samples):
    out = []
    for j in range(samples + 1):
        t = j / samples
        d = [list(p) for p in P]
        for r in range(1, len(d)):
            for i in range(len(d) - r):
                d[i] = _lerp(d[i], d[i + 1], t)
        out.append(d[0])
    return out


def path_points(P, interpolation, closed=False, samples=100, degree=3):
    if len(P) < 2:
        return []
    samples = max(2, int(samples))
    if interpolation == "catmull_rom":
        return _catmull_rom(P, closed, samples)
    if interpolation == "bspline":
        return _bspline(P, degree, closed, samples)
    if interpolation == "bezier":
        return _bezier(P, samples)
    return P + [P[0]] if closed else list(P)


# ====================================================================== geometry
def _quat_rpy(r, p, y):
    """Quaternion (x, y, z, w) of eTaSL roll, pitch, yaw: Rz(y) Ry(p) Rx(r)."""
    cr, sr, cp, sp, cy, sy = math.cos(r / 2), math.sin(r / 2), math.cos(p / 2), math.sin(p / 2), math.cos(y / 2), math.sin(y / 2)
    return (sr * cp * cy - cr * sp * sy, cr * sp * cy + sr * cp * sy, cr * cp * sy - sr * sp * cy, cr * cp * cy + sr * sp * sy)


def _quat_z_to(d):
    """Quaternion rotating the z axis onto direction d."""
    n = math.sqrt(sum(x * x for x in d)) or 1.0
    x, y, z = (v / n for v in d)
    if z < -0.999999:
        return (1.0, 0.0, 0.0, 0.0)
    q = (-y, x, 0.0, 1.0 + z)  # cross((0,0,1), d), 1 + dot
    m = math.sqrt(sum(c * c for c in q))
    return tuple(c / m for c in q)


def _rot(q, v):
    """Rotates v by the quaternion q (x, y, z, w)."""
    x, y, z, w = q
    R = [[1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
         [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
         [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]]
    return [sum(R[i][j] * v[j] for j in range(3)) for i in range(3)]


def _rgba(m):
    c = m.get("color", "#4e9af1")
    try:
        rgb = [int(c[i:i + 2], 16) / 255 for i in (1, 3, 5)]
    except (TypeError, ValueError, IndexError):
        rgb = [0.31, 0.6, 0.95]
    a = m.get("opacity", 0.4)
    return rgb + [min(1.0, max(0.0, float(a))) if isinstance(a, (int, float)) else 0.4]


def root_link(urdf):
    """Root link of a URDF (XML text or file path): the link that is not the child of a joint."""
    text = urdf
    if not urdf.lstrip().startswith("<"):
        with open(urdf, encoding="utf-8") as f:
            text = f.read()
    root = ET.fromstring(text)
    children = {j.find("child").get("link") for j in root.iter("joint") if j.find("child") is not None}
    links = [link.get("name") for link in root.iter("link")]
    return next((n for n in links if n not in children), None)


# ====================================================================== backend
class RVizBackend:
    def __init__(self, warn):
        self.warn = warn
        self.node = None
        self.pub = None
        self.base_frame = None
        self.base_warned = False
        self.base = []          # definitions loaded with load()
        self.patches, self.added, self.removed = {}, {}, set()
        self.published = {}     # name → (key of the last message, number of ids)
        self.trails = {}        # name → [(t, [x, y, z])]
        self.warned = set()
        self.desc_sub = None

    # ------------------------------------------------------------------ definitions (as MarkersPanel.compose/skillOp)
    def doc(self):
        ms = []
        for m in self.base:
            n = m.get("name")
            if n in self.removed or n in self.added:
                continue
            ms.append(self._patched(m))
        ms += [self._patched(m) for m in self.added.values()]
        return ms

    def _patched(self, m):
        out = dict(m)
        for k, v in self.patches.get(m.get("name"), {}).items():
            if v is None:
                out.pop(k, None)
            else:
                out[k] = v
        return out

    def _known(self, name, marker, props, warnings):
        t = type_of(marker)
        ok = set(COMMON) | set(TYPES.get(t, {}))
        out = {}
        for k, v in (props or {}).items():
            if k == "name" or k.startswith("is-"):
                warnings.append(f'"{name}": property "{k}" cannot be changed (another type: add() / MarkerEdit(type=...) replaces the marker)')
            elif k not in ok:
                warnings.append(f'"{name}": no property "{k}" for a marker of type {t} (ignored)')
            else:
                out[k] = v
        return out

    def op(self, op, name=None, props=None, type=None, **args):  # noqa: A002
        warnings = []
        find = lambda: next((m for m in self.doc() if m.get("name") == name), None)  # noqa: E731
        result = None
        if op == "update":
            m = find()
            if m is None:
                warnings.append(f'no marker named "{name}"' + ("" if self.base else " (outside the playground, markers must be added by the skill or loaded with load())"))
            else:
                self.patches.setdefault(name, {}).update(self._known(name, m, props, warnings))
        elif op == "add":
            # add or edit (as MarkersPanel.skillOp): a marker of this name and type is edited, else created (replacing
            # one of another type) with the given properties (the others: defaults)
            cur = find()
            if type not in TYPES:
                warnings.append(f'add "{name}": unknown marker type "{type}" ({", ".join(TYPES)})')
            elif cur is not None and type_of(cur) == type:
                self.patches.setdefault(name, {}).update(self._known(name, cur, props, warnings))
            else:
                m = {f"is-{type}": True, "name": name}
                m.update({k: v for k, v in self._known(name, m, props, warnings).items() if v is not None})
                self.added[name] = m
                self.patches.pop(name, None)
                self.removed.discard(name)
                self.trails.pop(name, None)
        elif op == "remove":
            if find() is None:
                warnings.append(f'no marker named "{name}"')
            else:
                self.removed.add(name)
                self.added.pop(name, None)
                self.patches.pop(name, None)
        elif op == "reset":
            if name is None:
                self.patches, self.added, self.removed = {}, {}, set()
            else:
                self.patches.pop(name, None)
                self.added.pop(name, None)
                self.removed.discard(name)
        elif op == "get":
            m = find()
            if m is None:
                warnings.append(f'no marker named "{name}"')
            return {"result": json.loads(json.dumps(m)) if m else None, "warnings": warnings}
        elif op == "names":
            return {"result": [m.get("name") for m in self.doc() if isinstance(m.get("name"), str)], "warnings": warnings}
        elif op == "freeze":
            pass  # values are already fixed outside the playground (links use their default)
        elif op == "load":
            try:
                with open(args["path"], encoding="utf-8") as f:
                    d = json.load(f)
                self.base = [m for m in d.get("markers", []) if isinstance(m, dict)]
            except Exception as e:  # noqa: BLE001
                warnings.append(f"load {args.get('path')}: {type(e).__name__}: {e}")
        else:
            warnings.append(f'unknown operation "{op}"')
        self.publish()
        return {"result": result, "warnings": warnings}

    # ------------------------------------------------------------------ ROS
    def init(self, node=None, base_frame=None, urdf=None):
        if node is not None:
            self.node = node
            self.pub = None
        if base_frame:
            self.base_frame = base_frame
        elif urdf:
            try:
                self.base_frame = root_link(urdf)
            except Exception as e:  # noqa: BLE001
                self.warn(f"init: cannot read the root link of the URDF: {type(e).__name__}: {e}")
        self.published.clear()
        self.publish()

    def _ready(self):
        if self.pub is not None:
            return True
        if self.node is None:
            try:
                from betfsm_ros import BeTFSMNode
                self.node = BeTFSMNode._instance  # the node of the skill, if it already exists
            except Exception:  # noqa: BLE001
                return False
            if self.node is None:
                return False
        try:
            from visualization_msgs.msg import Marker
            self.pub = self.node.create_publisher(Marker, "/visualization_marker", 10)
            self.node.create_timer(1.0, self._republish)
            if self.base_frame is None:
                self._subscribe_description()
        except Exception as e:  # noqa: BLE001
            self.warn(f"cannot publish RViz markers: {type(e).__name__}: {e}")
            self.pub = None
            return False
        return True

    def _subscribe_description(self):
        try:
            from rclpy.qos import DurabilityPolicy, QoSProfile
            from std_msgs.msg import String

            def cb(msg):
                try:
                    root = root_link(msg.data)
                except Exception:  # noqa: BLE001
                    return
                if root and root != self.base_frame:
                    self.base_frame = root
                    self.published.clear()
                    self.publish()

            self.desc_sub = self.node.create_subscription(
                String, "/robot_description", cb, QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
        except Exception:  # noqa: BLE001
            pass

    def _frame(self, ref):
        if ref in (None, "robot_base"):
            if self.base_frame is None:
                if not self.base_warned:
                    self.base_warned = True
                    self.warn('robot base frame not known yet (no /robot_description received): using "base_link"; '
                              'set it with init(base_frame=...) or init(urdf=...)')
                return "base_link"
            return self.base_frame
        return ref

    def _republish(self):
        self.published.clear()
        self.publish()

    def publish(self):
        if not self._ready():
            return
        from visualization_msgs.msg import Marker
        current = {}
        for m in self.doc():
            name = m.get("name")
            if not isinstance(name, str):
                continue
            msgs = self._messages(m) if m.get("enabled", True) is not False else []
            key = json.dumps([m, self.base_frame, len(self.trails.get(name, []))], default=str)
            current[name] = (key, len(msgs))
            last = self.published.get(name)
            if last is not None and last[0] == key:
                continue
            for i, msg in enumerate(msgs):
                msg.ns, msg.id = name, i
                self.pub.publish(msg)
            for i in range(len(msgs), last[1] if last else 0):
                self.pub.publish(Marker(ns=name, id=i, action=Marker.DELETE))
        for name, (_, count) in self.published.items():
            if name not in current:
                for i in range(count):
                    self.pub.publish(Marker(ns=name, id=i, action=Marker.DELETE))
        self.published = current

    # ------------------------------------------------------------------ translation
    def _once(self, name, text):
        if (name, text) not in self.warned:
            self.warned.add((name, text))
            self.warn(f'RViz: "{name}": {text}')

    def _value(self, m, key):
        """Value of a property (default of the type if absent; the default of a link, else None)."""
        t = type_of(m)
        v = m.get(key, TYPES[t].get(key))
        if is_link(v):
            if "default" in v:
                self._once(m["name"], f'property "{key}" is an eTaSL expression ({v["etasl"]}): its default is used outside the playground')
                return v["default"]
            self._once(m["name"], f'property "{key}" is an eTaSL expression ({v["etasl"]}) without default: not available '
                                  f'outside the playground{" (not drawn)" if key == "position" else ""}')
            return None
        if key == "points" and isinstance(v, list):
            pts = []
            for p in v:
                if is_link(p):
                    if "default" in p:
                        pts.append(p["default"])
                        self._once(m["name"], f'point {p["etasl"]} is an eTaSL expression: its default is used outside the playground')
                    else:
                        self._once(m["name"], f'point {p["etasl"]} is an eTaSL expression without default: left out outside the playground')
                else:
                    pts.append(p)
            return pts
        return v

    def _base(self, m):
        from visualization_msgs.msg import Marker
        msg = Marker()
        msg.header.frame_id = self._frame(m.get("reference"))
        try:
            msg.header.stamp = self.node.get_clock().now().to_msg()
        except Exception:  # noqa: BLE001
            pass
        msg.action = Marker.ADD
        msg.pose.orientation.w = 1.0
        r, g, b, a = _rgba(m)
        msg.color.r, msg.color.g, msg.color.b, msg.color.a = r, g, b, a
        return msg

    @staticmethod
    def _set_pose(msg, p, q=(0.0, 0.0, 0.0, 1.0)):
        msg.pose.position.x, msg.pose.position.y, msg.pose.position.z = (float(x) for x in p[:3])
        msg.pose.orientation.x, msg.pose.orientation.y, msg.pose.orientation.z, msg.pose.orientation.w = q

    @staticmethod
    def _points(msg, pts):
        from geometry_msgs.msg import Point
        msg.points = [Point(x=float(p[0]), y=float(p[1]), z=float(p[2])) for p in pts]

    def _messages(self, m):
        from visualization_msgs.msg import Marker
        t = type_of(m)
        name = m["name"]
        if t is None:
            self._once(name, "no marker type: not drawn")
            return []
        if t == "cone":
            self._once(name, "cone markers are not supported in RViz: not drawn")
            return []
        if m.get("visibility", "always") != "always":
            self._once(name, f'visibility "{m["visibility"]}" is ignored outside the playground (always drawn)')
        if t == "path":
            pts = self._value(m, "points") or []
            pts = [p for p in pts if isinstance(p, list) and len(p) >= 3]
            if len(pts) < 2:
                self._once(name, "fewer than 2 points available: not drawn")
                return []
            interp = m.get("interpolation", "polyline")
            curve = path_points(pts, interp, bool(m.get("closed", False)), m.get("samples", 100), m.get("degree", 3))
            line = self._base(m)
            line.type = Marker.LINE_STRIP
            line.scale.x = max(float(self._value(m, "width") or 0), 0.002)
            self._points(line, curve)
            out = [line]
            if m.get("show_points"):
                balls = self._base(m)
                balls.type = Marker.SPHERE_LIST
                d = max(3 * line.scale.x, 0.01)
                balls.scale.x = balls.scale.y = balls.scale.z = d
                self._points(balls, pts)
                out.append(balls)
            return out
        p = self._value(m, "position")
        if not (isinstance(p, list) and len(p) >= 3):
            if not is_link(m.get("position")):  # (a link without default was already reported)
                self._once(name, "no position available: not drawn")
            return []
        msg = self._base(m)
        if t in ("sphere", "point"):
            msg.type = Marker.SPHERE
            d = 2 * float(self._value(m, "radius") or 0.05) if t == "sphere" else float(self._value(m, "size") or 0.02)
            msg.scale.x = msg.scale.y = msg.scale.z = d
            self._set_pose(msg, p)
        elif t == "box":
            msg.type = Marker.CUBE
            s = self._value(m, "size") or [0.1, 0.1, 0.1]
            msg.scale.x, msg.scale.y, msg.scale.z = (float(x) for x in s[:3])
            self._set_pose(msg, p, _quat_rpy(*(self._value(m, "orientation") or [0, 0, 0])[:3]))
        elif t == "cylinder":
            msg.type = Marker.CYLINDER
            r, length = float(self._value(m, "radius") or 0.05), float(self._value(m, "length") or 0.3)
            msg.scale.x, msg.scale.y, msg.scale.z = 2 * r, 2 * r, length
            self._set_pose(msg, p, _quat_z_to(self._value(m, "axis") or [0, 0, 1]))
        elif t == "plane":
            msg.type = Marker.CUBE
            s = self._value(m, "size") or [0.4, 0.4]
            msg.scale.x, msg.scale.y, msg.scale.z = float(s[0]), float(s[1]), 0.001
            self._set_pose(msg, p, _quat_z_to(self._value(m, "normal") or [0, 0, 1]))
        elif t == "arrow":
            msg.type = Marker.ARROW
            v = self._value(m, "vector") or [0, 0, 0.1]
            s = float(self._value(m, "scale") or 1)
            end = [p[i] + s * v[i] for i in range(3)]
            length = math.dist(p[:3], end)
            msg.scale.x, msg.scale.y, msg.scale.z = max(length * 0.05, 0.003), max(length * 0.1, 0.006), 0.0
            self._points(msg, [p, end])
        elif t == "line":
            msg.type = Marker.LINE_STRIP
            msg.scale.x = 0.004
            end = self._value(m, "end")
            if not (isinstance(end, list) and len(end) >= 3):
                self._once(name, "no end point available: not drawn")
                return []
            self._points(msg, [p, end])
        elif t == "trail":
            msg.type = Marker.LINE_STRIP
            msg.scale.x = 0.003
            now = time.monotonic()
            trail = self.trails.setdefault(name, [])
            if not trail or trail[-1][1] != p[:3]:
                trail.append((now, list(p[:3])))
            duration = float(self._value(m, "duration") or 10)
            trail[:] = [x for x in trail if now - x[0] <= duration]
            if len(trail) < 2:
                return []
            self._points(msg, [x[1] for x in trail])
        elif t == "text":
            msg.type = Marker.TEXT_VIEW_FACING
            msg.text = str(self._value(m, "text") or "")
            msg.scale.z = float(self._value(m, "size") or 0.04)
            self._set_pose(msg, p)
        elif t == "frame":
            size = float(self._value(m, "size") or 0.1)
            q = _quat_rpy(*(self._value(m, "orientation") or [0, 0, 0])[:3])
            out = []
            for axis, rgb in (([1, 0, 0], (1.0, 0.0, 0.0)), ([0, 1, 0], (0.0, 1.0, 0.0)), ([0, 0, 1], (0.0, 0.0, 1.0))):
                a = self._base(m)
                a.type = Marker.ARROW
                a.scale.x, a.scale.y, a.scale.z = size * 0.05, size * 0.1, 0.0
                a.color.r, a.color.g, a.color.b = rgb
                a.color.a = 1.0
                d = _rot(q, axis)
                self._points(a, [p, [p[i] + size * d[i] for i in range(3)]])
                out.append(a)
            return out
        return [msg]
