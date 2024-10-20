# Etasl ROS2 Application Template

TODO

Notes Erwin:
- parameters in blackboard ["tasks][taskname]

## Dependencies

dkjson is used to generate a pretty json schema for each of the task specifications. To install it use luarocks:

```bash
sudo luarocks install dkjson
```

(The standard json library does not support pretty encoding)


## Questions and remarks from Erwin

- Does not make sense to publish on /fsm/event, it makes much more sense to publish on /etasl_node/event.  The statemachine needs to be able to differentiate between possibly different etasl nodes that
are running.

- parameters of tasks:  why do you adapt them with "constant( ...)" ?  They are more powerfull if they
  are normal numbers (useable in "if" statements e.g.).  Users can always do "constant(..)" themselves.
  **I've adapted the code to return values and not expressions, this allows to use nil to communicate
  missing optional values, and also use exactly the same mechanism for createEnumeratedParameter()**

- defined **createEnumeratedParameter** to define enumerated string parameters.  e.g. to indicate units or indicate
alternative ways to execute the same skill, add additional constraints, etc...

- Missing some way to deal with orientations and frames:  use SE3.py/SO3.py/quaternion.py/dualquaternion.py python libraries from robotgenskill?
  Would recommend quaternions for transfer to eTaSL




