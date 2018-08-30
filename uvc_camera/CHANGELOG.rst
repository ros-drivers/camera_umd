^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package uvc_camera
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.2.7 (2018-08-30)
------------------
* Merge pull request `#22 <https://github.com/ros-drivers/camera_umd/issues/22>`_ from ros-drivers/fix_9
  install launch files
* install launch files
* Contributors: Kei Okada

0.2.6 (2018-08-30)
------------------
* Merge pull request `#21 <https://github.com/ros-drivers/camera_umd/issues/21>`_ from ros-drivers/add_travis
  add .travis.yml including melodic
* Merge remote-tracking branch 'mikaelarguedas/patch-1' into add_travis
* update to use non deprecated pluginlib macro
* Contributors: Kei Okada, Mikael Arguedas

0.2.5 (2017-06-15)
------------------
* add ROS Orphaned Package Maintainers to maintainer tag (`#18 <https://github.com/ros-drivers/camera_umd/pull/18>`_)
* important property: focus_absolute ; add because here example == documentation
* Added exposure, gain, horizontal flip and vertical flip controls (`#15 <https://github.com/ros-drivers/camera_umd/pull/15>`_)
* Added support for some camera controls in ROS .launch files.  (`#14 <https://github.com/ros-drivers/camera_umd/pull/14>`_)
  * Added exposure, gain, horizontal flip and vertical flip controls
  * Added support for camera controls in ROS .launch files. Also added example.launch
* Support MJPEG format direct streaming (`#13 <https://github.com/ros-drivers/camera_umd/pull/13>`_)
* Add comment in launchfiles. (`#12 <https://github.com/ros-drivers/camera_umd/pull/12>`_)
* Add brightness control parameter. (`#12 <https://github.com/ros-drivers/camera_umd/pull/12>`_)
* Contributors: Adrian Yuen, Glass Bot, Kei Okada, Lingzhu Xiang, Toni Oliver

0.2.4 (2014-06-29)
------------------
* Added new parameters: auto_focus (bool), focus_absolute (int), auto_exposure (bool),
  exposure_absolute (int), power_line_frequency (int: 0/50/60)
* Contributors: Andreas Bihlmaier

0.2.3 (2014-06-26)
------------------
* Disabled hardcoded default parameters.
* Contributors: Ken Tossell

0.2.2 (2014-06-24)
------------------
* Print warnings instead of crashing when camera features are unavailable.
* Support V4L drivers that need user-writable mmap regions (e.g., bttv).
* Fixed nodelet names in the launch files.
* Added camera_name parameter, sent to camera_info_manager.
* Contributors: James Sarrett, Ken Tossell

0.2.1 (2014-01-12)
------------------
* Added install rule for nodelet_uvc_camera.xml

0.2.0 (2013-08-02)
------------------
* Converted uvc_camera to catkin
