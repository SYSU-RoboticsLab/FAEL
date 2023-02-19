# Total number of satellites in view and data about satellites
# Because the NMEA sentence is limited to 4 satellites per message, several
# of these messages may need to be synthesized to get data about all visible
# satellites.

Header header

string message_id

# Number of messages in this sequence
uint8 n_msgs
# This messages number in its sequence. The first message is number 1.
uint8 msg_number

# Number of satellites currently visible
uint8 n_satellites

# Up to 4 satellites are described in each message
GpgsvSatellite[] satellites
