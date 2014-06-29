^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package uvc_camera
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

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
