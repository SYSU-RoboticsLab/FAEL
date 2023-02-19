^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package jackal_control
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.6.5 (2020-04-20)
------------------

0.6.4 (2020-03-04)
------------------
* Update control.launch
  Somehow a ">" has gone missing. This change adds it back in.
* [jackal_control] Added control extras.
* Contributors: Jeff Schmidt, Tony Baltovski

0.6.3 (2019-07-18)
------------------

0.6.2 (2019-04-18)
------------------

0.6.1 (2018-08-02)
------------------
* Added missing twist_mux.yaml.  Fixed malformed meshes
* Added twist_mux and minor kinetic syntax changes
* Contributors: Dave Niewinski

0.6.0 (2018-04-12)
------------------
* [jackal_control] Made the PS4 controller default.
* Made minor changes to syntax for kinetic warnings
* Contributors: Dave Niewinski, Tony Baltovski

0.5.4 (2018-04-12)
------------------

0.5.3 (2016-06-01)
------------------
* Added support for PS4 controller.
* Contributors: Tony Baltovski

0.5.2 (2016-02-10)
------------------
* Removed reference to FootprintLayer.
* Increased inflation radius to account for uneven wall in Jackal_world.
* Added pointgrey camera and accessories.
* Improve robot_localiztion params
* Added Sick LMS1XX URDF.
* Fixed example calibration output.
* Added tutorials.
* Contributors: Mike Purvis, Martin Cote, Tony Baltovski, James Servos


0.5.1 (2015-02-02)
------------------

0.5.0 (2015-01-20)
------------------
* Support disabling the joystick for simulation when only the interactive markers are desired for teleop.
* Contributors: Mike Purvis

0.4.2 (2015-01-14)
------------------
* Shorten timeout for the controller spawner's shutdown.
* Contributors: Mike Purvis

0.4.1 (2015-01-07)
------------------
* Remove fork of diff_drive_controller.
* Contributors: Mike Purvis

0.4.0 (2014-12-12)
------------------
* added joystick argumant.
* Adding imu0_differential setting (=true) to control.yaml
* Add dep for joint state controller.
* Contributors: Mike Purvis, Shokoofeh Pourmehr, Tom Moore

0.3.0 (2014-09-10)
------------------

0.2.1 (2014-09-10)
------------------
* Depend on diff_drive_controller.
* Contributors: Mike Purvis

0.2.0 (2014-09-09)
------------------
* Add fork of diff_drive_controller.
* Fix run_depend elements.
* Fix remap for the interactive markers.
* New jackal_control package.
  This is launchers and configuration common to simulated and real
  Jackal, including controller, localization, and teleop.
* Contributors: Mike Purvis
