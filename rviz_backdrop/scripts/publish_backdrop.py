#!/usr/bin/env python

import roslib; roslib.load_manifest( 'rviz_plugin_tutorials' )
from sensor_msgs.msg import Image
from sensor_msgs.msg import CompressedImage
import rospy

# image_file = 'theater_red_curtains-wide.jpg'
image_file = 'workshop-short.jpg'

# Note: this is not doing a complete proper ROS image transport
# publisher.  That doesn't support Python.  Instead, I'm publishing
# directly on the /backdrop/compressed topic a
# sensor_msgs/CompressedImage message, with the raw jpeg data just
# loaded into it.  This way my python file doesn't have to decode or
# convert the image data at all.  It does mean when you ask RViz for
# the available image topics, this won't show up, you'll have to type
# "/backdrop" directly into the topic field.  (RViz adds
# "/compressed".)

pub = rospy.Publisher( '/backdrop/compressed', CompressedImage )

rospy.init_node( 'backdrop_node' )

rate = rospy.Rate(1)

dist = 3

image = CompressedImage()
image.header.frame_id = "/odom_combined"
image.format = "jpeg"
image.data = open( image_file ).read()

while not rospy.is_shutdown():

    image.header.stamp = rospy.Time.now()

    pub.publish( image )

    print "published."
    rate.sleep()
