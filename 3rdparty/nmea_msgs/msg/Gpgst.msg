# Message from GPGST NMEA String
Header header

string message_id

# UTC seconds from midnight
float64 utc_seconds

# Root-Mean-Squared value of standard deviation of range inputs
float32 rms

# Standard Deviations of semi-major and minor axes of error ellipse
float32 semi_major_dev
float32 semi_minor_dev

# Orientation of the semi-major axis of error ellipse with respect to true north
float32 orientation

# Standard Deviations of latitude, longitude, and altitude measurements
float32 lat_dev
float32 lon_dev
float32 alt_dev
