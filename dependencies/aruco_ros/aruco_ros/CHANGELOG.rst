^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package aruco_ros
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.2.4 (2019-08-28)
------------------
* Remove build warnings
* Merge pull request `#65 <https://github.com/pal-robotics/aruco_ros//issues/65>`_ from 7675t/add_rotate_axis_param
  Add rotate_marker_axis parameter
* Add rotate_marker_axis parameter
* Contributors: Ryosuke Tajima, Victor Lopez

0.2.3 (2018-04-20)
------------------
* Force marker_publisher to build after all dependencies.
* Fixed OpenCV Calib3D link error on ROS Kinetic
* Add aruco_ros_utils lib and fix some missing dependencies
* Replace assert by error message to keep library functional
* Contributors: Bence Magyar, Christopher Hrabia, Jordi Pages, Ugnius Malūkas, Victor Lopez, Voidminded, ethanfowler

0.2.2 (2017-07-25)
------------------
* only look for aruco if someone is looking for them
* Contributors: Victor Lopez

0.2.1 (2017-07-21)
------------------
* Change default threshold to match defaults of aruco marker detector
* Add dynamic reconfigure to simple_single
* Contributors: Victor Lopez

0.2.0 (2016-10-19)
------------------
* only proccesses images if there are subscribers
* add rviz marker and add corner param
* use double precision to improve accuracy
* Contributors: Jordi Pages, Procópio Stein

0.1.0 (2015-08-10)
------------------
* Update changelogs and maintainer email
* Frame parameters only checked when using camera info
* Add marker list publisher
* Remove unused broadcaster
* Only do 3d when there is camera info
* Use waitForMessage for camerainfo
* Remove nonsense assert
* Reorganize and allow no camera_info
* Fix crash when distortion vector is 0 long (usb_cam)
* Contributors: Bence Magyar

0.0.1 (2015-05-20)
------------------
* More accurate ROS timestamps (callback triggering time)
  This commit ensures that:
  - all published msgs in a callback have the same timestamp
  - the time is as close as possible to the frame grabbing time (as fast as the marker detection may be, the delay might affect TF interpolation in an unacceptable way for applications like visual servoing)
* Install marker_publisher executable
  This target was missing in the installation rule
* Finished some renaming
* changes to finish branch merge
* aruco_ros: Fixing superfluous (and broken) linker arg to -laruco
* Reorganize aruco_ros into 3 packages
* Contributors: Bence Magyar, Jordi Pages, Josh Langsfeld, ObiWan, Steve Vozar
