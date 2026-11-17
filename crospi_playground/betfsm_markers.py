"""
BeTFSM states that change the markers of the Crospi Playground (and, outside the playground, the RViz markers
published by playground_markers): use them in a skill's tree like any other BeTFSM state.

    from crospi_playground import playground_markers as mk
    from crospi_playground.betfsm_markers import MarkerEdit, MarkerUpdater, WithMarkers

    Sequence("demo", [
        MarkerEdit("goal red", "goal", color="#e15759"),                                     # edit a marker
        MarkerEdit("add goal", "goal", type="sphere", position=[0.5, 0, 0.3], radius=0.02),  # add (or edit) a marker
        MarkerEdit("add goal", json='{"is-sphere":true,"name":"goal","position":[0.5,0,0.3],"radius":0.02}'),  # same
        WithMarkers("move", CrospiTask("MoveToGoal", "MoveToGoal"),                          # changes during a state
                    {"goal": {"color": "#e15759"}, "tool": {"type": "frame", "position": mk.expr("tcp")}}),
        Concurrent("wait", [TimedWait("pause", 2.0),                                         # continuous updates
                            MarkerUpdater("countdown", "label", lambda bb: f"{bb['left']:.1f} s", prop="text")]),
    ])

Markers are found by their name. With a type (type="sphere", or "is-sphere" in the JSON), a marker with this name and
type is edited (only the given properties change); otherwise it is created with the given properties, the others take
their defaults (a marker of another type with this name is replaced). Without a type, the marker must exist (e.g. in the
Markers tab) and is edited.

JSON: the one of a marker of the Markers tab ("Copy JSON" in its Form view copies it on one line), e.g.
json='{"is-sphere":true,"name":"goal","color":"#59a14f","radius":0.02}': its "is-<type>" gives the type and its "name"
the marker (when not given); the keyword arguments override its values (e.g. a position computed from the blackboard).

Values of the properties can be plain values, mk.expr(...) links, or computed when the state executes: a callable
`f(blackboard)` or a BeTFSM ConfigCallback (reset(state, blackboard), then called with (state, blackboard)). Values
inside lists and dicts are resolved too (e.g. points=[[0, 0, 0], lambda bb: bb["target"]]).
"""
import json as _json
import time

from betfsm import SUCCEED, TICKING, ConfigCallback, Generator, GeneratorWithState, get_logger

from crospi_playground import playground_markers as mk

_ACTIONS = ("type", "remove", "reset", "freeze")


def resolve(value, state, blackboard):
    """Value of a property: callables and ConfigCallbacks are evaluated (recursively in lists and dicts)."""
    if isinstance(value, ConfigCallback):
        if not value.reset(state, blackboard):
            return None
        return mk._plain(value(state, blackboard))
    if callable(value):
        return mk._plain(value(blackboard))
    if isinstance(value, list):
        return [resolve(v, state, blackboard) for v in value]
    if isinstance(value, tuple):
        return [resolve(v, state, blackboard) for v in value]
    if isinstance(value, dict):
        return {k: resolve(v, state, blackboard) for k, v in value.items()}
    return value


def from_json(text):
    """
    A marker as JSON (a string, or a dict) → (name, change): its "is-<type>" becomes "type", the other keys are its
    properties. Raises ValueError for a JSON that is not one marker.
    """
    m = _json.loads(text) if isinstance(text, str) else dict(text or {})
    if not isinstance(m, dict):
        raise ValueError("the JSON of a marker must be an object, e.g. {\"is-sphere\": true, \"name\": \"goal\", ...}")
    types = [k[3:] for k, v in m.items() if k.startswith("is-") and v]
    change = {k: v for k, v in m.items() if not k.startswith("is-") and k != "name"}
    if types:
        change["type"] = types[0]
    return m.get("name"), change


def _change(change):
    """A change given as a dict or as the JSON of a marker (string)."""
    if isinstance(change, str):
        return from_json(change)[1]
    return dict(change or {})


def _apply(name, change, state, blackboard):
    """Applies one change {"type": type?, "remove": bool?, "reset": bool?, "freeze": bool?, props...}."""
    change = dict(change)
    kind = change.pop("type", None)
    remove, reset, freeze = change.pop("remove", False), change.pop("reset", False), change.pop("freeze", False)
    props = {k: resolve(v, state, blackboard) for k, v in change.items()}
    if reset:
        mk.reset(name)
    if remove:
        mk.remove(name)
        return
    if kind:
        mk.add(name, kind, props)  # add or edit
    elif props:
        mk.update(name, props)
    if freeze:
        mk.freeze(name)


class MarkerEdit(Generator):
    """
    One-shot change of a marker; returns SUCCEED immediately (no tick).

        MarkerEdit("show target", "target", enabled=True)                         # edit properties
        MarkerEdit("add goal", "goal", type="sphere", position=lambda bb: bb["goal"], radius=0.02)   # add or edit
        MarkerEdit("add goal", json='{"is-sphere":true,"name":"goal","radius":0.02,"color":"#59a14f"}',
                   position=lambda bb: bb["goal"])                                # JSON of the Markers tab + arguments
        MarkerEdit("drop pin", "contact", freeze=True, color="#59a14f")           # links → current values
        MarkerEdit("pin", lambda bb: f"visited {bb['k']}", type="point", position=mk.expr("origin(task_frame)"), freeze=True)
        MarkerEdit("remove goal", "goal", remove=True)
        MarkerEdit("back to the tab", "target", reset=True)                       # marker=None: all markers

    Parameters:
        name: name of the state
        marker: name of the marker (or a callable f(blackboard) giving it); default: the "name" of the JSON
        props: properties (dict)
        json: the marker as JSON (string or dict), e.g. copied from the Markers tab: "is-<type>" gives the type,
            "name" the marker (if not given); props and the keyword arguments override its values
        type: marker type (sphere, cylinder, box, plane, cone, point, frame, arrow, line, trail, text, path): the marker
            is added, or edited if one with this name and type exists. Without a type, an existing marker is edited.
        remove: removes the marker
        reset: drops the changes of the skill to the marker (before the other changes)
        freeze: then replaces its eTaSL expressions by their current values (e.g. add a marker at an expression and
            freeze it: a pin where the expression is now)
        **kw: properties (values, mk.expr links, callables f(blackboard) or ConfigCallbacks)
    """

    def __init__(self, name, marker=None, props=None, *, json=None, type=None, remove=False, reset=False,  # noqa: A002
                 freeze=False, **kw):
        super().__init__(name, [SUCCEED])
        base = {}
        if json is not None:
            json_name, base = from_json(json)
            if marker is None:
                marker = json_name
        if marker is None and not reset:
            raise ValueError(f"MarkerEdit '{name}': the marker (its name) is missing (argument marker or \"name\" of the JSON)")
        self.marker = marker
        self.change = {**base, **(props or {}), **kw, "remove": remove, "reset": reset, "freeze": freeze}
        if type is not None:
            self.change["type"] = type

    def co_execute(self, blackboard):
        if self.marker is None and self.change.get("reset"):
            mk.reset()
        else:
            marker = self.marker(blackboard) if callable(self.marker) else self.marker
            _apply(marker, self.change, self, blackboard)
        yield SUCCEED


class MarkerUpdater(Generator):
    """
    Updates a property of a marker continuously from a getter, as BeTFSM's MarkerPublisher; never finishes by itself
    (use it in a Concurrent, ConcurrentSequence, While, ... next to the states it illustrates).

        Concurrent("move and show force", [
            CrospiTask("guarded_motion_z", "guarded_motion_z"),
            MarkerUpdater("force", "force arrow", ForceCallback(), prop="vector", frequency=20),
        ])

    Parameters:
        name: name of the state
        marker: name of the marker
        getter: ConfigCallback (reset(state, blackboard) until True, then called with (state, blackboard)) or callable
            f(blackboard); its value is written into `prop`, or, if it is a dict, into those properties
        prop: property updated (default "position")
        frequency: updates per second [Hz] (simulated time in the playground)
        on_exit: when the state ends or is stopped: None (keep), "hide", "reset" (back to before the skill changed
            it), "remove" or "freeze" (links → current values)
    """

    def __init__(self, name, marker, getter, prop="position", frequency=10.0, on_exit=None):
        super().__init__(name, [SUCCEED])
        self.marker, self.getter, self.prop = marker, getter, prop
        self.period = 1.0 / frequency if frequency > 0 else 0.0
        self.on_exit = on_exit
        self.running = False

    def _value(self, blackboard):
        if isinstance(self.getter, ConfigCallback):
            return mk._plain(self.getter(self, blackboard))
        return mk._plain(self.getter(blackboard))

    def co_execute(self, blackboard):
        if isinstance(self.getter, ConfigCallback):
            logged = False
            while not self.getter.reset(self, blackboard):
                if not logged:
                    get_logger().warning(f"MarkerUpdater '{self.name}': configuration error in callback (ConfigCallback)")
                    logged = True
                yield TICKING
        self.running = True
        last = None
        while True:
            now = time.monotonic()
            if last is None or now - last >= self.period:
                last = now
                value = self._value(blackboard)
                if value is not None:
                    mk.update(self.marker, value if isinstance(value, dict) else {self.prop: value})
            yield TICKING

    def exit(self):
        if self.running:
            self.running = False
            if self.on_exit == "hide":
                mk.hide(self.marker)
            elif self.on_exit == "reset":
                mk.reset(self.marker)
            elif self.on_exit == "remove":
                mk.remove(self.marker)
            elif self.on_exit == "freeze":
                mk.freeze(self.marker)
        return super().exit()


class WithMarkers(GeneratorWithState):
    """
    Runs a state with changed markers, and undoes the changes when it ends (in any way: finished, failed, stopped by
    a Concurrent or a While, ...). Same outcomes as the state.

        WithMarkers("approach", CrospiTask("approach", "approach"), {
            "target": {"color": "#e15759", "radius": 0.03},           # changed properties: restored afterwards
            "tool": {"type": "frame", "position": mk.expr("tcp")},    # added marker: removed afterwards
            "hint": {"enabled": False},                               # hidden during the state
            "goal": '{"is-sphere":true,"name":"goal","radius":0.05}', # the JSON of a marker (Markers tab)
        })

    Parameters:
        name: name of the state
        state: the state
        changes: {marker name: change}; a change is a dict of properties (values as in MarkerEdit; "type": add or edit
            a marker of that type) or the JSON of a marker (string)
    """

    def __init__(self, name, state, changes):
        super().__init__(name, [], state)
        self.changes = {marker: _change(change) for marker, change in changes.items()}
        self.undo = None

    def co_execute(self, blackboard):
        self.undo = []
        existing = set(mk.names() or [])
        for marker, change in self.changes.items():
            before = mk.get(marker) if marker in existing else None  # (get() warns about a missing marker)
            if before is None:
                self.undo.append((marker, None, None))  # added: removed afterwards
            else:
                kind = next((k[3:] for k, v in before.items() if k.startswith("is-") and v), None)
                if change.get("type") not in (None, kind):
                    # replaced by a marker of another type: the whole marker is restored afterwards
                    self.undo.append((marker, kind, {k: v for k, v in before.items() if not k.startswith("is-") and k != "name"}))
                else:
                    # edited: the properties that change are restored afterwards (None: absent before, removed again)
                    self.undo.append((marker, "", {k: before.get(k) for k in change if k not in _ACTIONS}))
            _apply(marker, change, self, blackboard)
        outcome = self.state(blackboard)
        while outcome == TICKING:
            yield TICKING
            outcome = self.state(blackboard)
        yield outcome

    def exit(self):
        if self.undo is not None:
            for marker, kind, props in reversed(self.undo):
                if kind is None:
                    mk.remove(marker)
                elif kind:
                    mk.add(marker, kind, props)  # another type: replaces the marker by the original one
                elif props:
                    mk.update(marker, props)
            self.undo = None
        return super().exit()
