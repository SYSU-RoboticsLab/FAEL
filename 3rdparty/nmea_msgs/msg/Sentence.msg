# A message representing a single NMEA0183 sentence.

# header.stamp is the ROS Time when the sentence was read.
# header.frame_id is the frame of reference reported by the satellite
#        receiver, usually the location of the antenna.  This is a
#        Euclidean frame relative to the vehicle, not a reference
#        ellipsoid.
Header header

# This should only contain ASCII characters in order to be a valid NMEA0183 sentence.
string sentence
