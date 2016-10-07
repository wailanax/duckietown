## Duckietown Source Code Notes
This document is still a work in progress...
I am trying to go through each package to see what they contain.

-----

*  adafruit_drivers
 
   packages for the HATS, as well as several other componenets they seem to support using. 

*  adafruit_imu
  
   similar package for an optional IMU

*  apriltags_ros
   
   an implementation of AprilTags for ROS  
   April tags are the same as AR tags or Alvar Tags... just a different implementation.  
   Developed by Prof Edwin Olson of University of Michigan ( http://april.eecs.umich.edu )  
   Code was ported to C++ ( http://wiki.tekkotsu.org/index.php/AprilTags )  
   Code has now been modified (by MIT) to be a standalone library with homography based on openCV.   

   It may be possible to use some other library if a better one exists or if we want to standardize.  

  *  apriltags
 
    The c++ library implementation  
    The tag "families"  

  *  apriltags_ros
      
     The ROS node implementation (written in cpp)  
     Adds
     * apriltag\_detector\_node
     * apriltags\_postprocessing\_node

*   _attic
	
    Old code they didn't want to get rid of but is no longer in use...




