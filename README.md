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


## Questions from Erwin


- It seems that ServiceManager is not really used.  Is it to cache/reuse the create_client calls when using a ServiceState?
  (so work in progress and ServiceState still needs to be adapted?)

- Missing some way to deal with orientations and frames:  use SE3.py/SO3.py/quaternion.py/dualquaternion.py python libraries from robotgenskill?
  Would recommend quaternions for transfer to eTaSL

