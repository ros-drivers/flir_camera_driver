# How to develop your own camera configuration file

The camera configuration file defines the available ROS parameters,
and how they relate to the corresponding [Spinnaker
nodes](https://www.flir.com/support-center/iis/machine-vision/application-note/spinnaker-nodes/).
The Spinnaker API follows the GenICam standard, where each property
(e.g. exposure mode, gain, ...) of the camera is represented by a
node. Many properties are of integer or floating point type, but some
are enumerations ("enum"). Before you modify a configuration file you
can explore the Spinnaker Nodes by using the spinview applications
that comes with the Spinnaker SDK. Once you know what property you
want to expose as a ROS parameter, you add a mapping entry to the yaml
configuration file, e.g.:
```yaml
  - name: image_width
    type: int
    node: ImageFormatControl/Width
```

With this entry in place, the ROS driver will now accept an integer parameter
of name ``image_width``, and whenever
``image_width`` changes, it will apply this change to the Spinnaker
Node ``ImageFormatControl/Width``.

Enumerations (``enum``) parameters are slightly trickier than float
and integers because their values are restricted to a set of
strings. Any other strings will be rejected by the Spinnaker API.
Please document the valid enum strings in the configuration file,
e.g.:
```yaml
  - name: line1_linemode  # valid values: "Input", "Output"
    type: enum
    node: DigitalIOControl/LineMode
```

The hard part is often finding the node name, in the last
example ``"DigitalIOControl/LineMode"``. It usually follows by
removing spaces from the ``spinview`` names. If that doesn't work,
launch the driver with the ``dump_node_map`` parameter set to "True"
and look at the output for inspiration.

**NOTE: !!!!  THE ORDER OF PARAMETER DEFINITION MATTERS !!!!**

On node startup, the parameters will be declared and initialized
in the order listed in the yaml file. For instance you must list
the enum ``exposure_auto`` before the float ``exposure_time`` because on
startup, ``exposure_auto`` must first be set to ``Off`` before
``exposure_time`` can be set, or else the camera refuses to set
the exposure time.
