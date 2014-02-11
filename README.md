ros_markers
===========

This project is a ROS wrapper for the EPFL's chilitags library.

![Nao looking at markers](doc/nao_markers.jpg)

The [chilitags](http://chili.epfl.ch/software) C++ library is a robust
and lightweight library for detection of fiducial markers.

Main features:
- recognizes up to 1024 markers simultaneously in a scene, 
- has a focus on robustness to bad light conditions, 
- provides (filtered) 6D localization of the markers,
- supports objects with multiple markers (with increased robustness),
- OpenCV (>=2.3) is the only dependency.

This ROS node wraps the chilitags library to use the standard ROS mechanisms:
images and camera calibration are read from a standard ROS camera, and 6D
position of markers are published as TF transforms.

The most basic usage is:

```
$ rosrun ros_markers detect image:=/camera/image_raw
```

This will start to look for markers in the image stream, and publish the TF
transformation of the detected ones.

The provided [launch file](launch/detect.launch) list all the available
parameters. The most important one is the *markers configuration*.  This (YAML)
file describes where are the markers on your objects, and let you publish the
position of an object with multiple markers attached. See [the sample
configuration](config/markers_configuration_sample.yml) for a complete example.

Other resources
---------------

- This project is very similar to [ROS
ALVAR](http://wiki.ros.org/ar_track_alvar). ALVAR has a broader scope (like
support for integration with depth maps), but it is also heavier. Benchmarks
have yet to be conducted.
- The standard (but much older) tool is ARToolkit. It also has [ROS
  bindings](http://wiki.ros.org/artoolkit).

