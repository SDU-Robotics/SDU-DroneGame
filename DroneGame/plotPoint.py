

#!/usr/bin/env python
# license removed for brevity
import rospy
from std_msgs.msg import String
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point

class Relay:
	def __init__(self):
	    rospy.init_node('talker', anonymous=True)
	    rospy.Subscriber("positionPublisher5", Point, self.callback)
	    self.markerpub = rospy.Publisher('chatter', Marker, queue_size=10)

	def spin(self):
		rospy.spin()

	def callback(self, msg):
		marker = Marker()
		marker.type = Marker.POINTS
		marker.header.frame_id = "map"
		marker.color.r = 1.0
		marker.color.g = 0.0
		marker.color.b = 0.0
		marker.color.a = 1.0

		marker.scale.x = 0.01
		marker.scale.y = 0.01
		marker.scale.z = 0.01

		marker.points = []
		marker.points.append(msg)
	        self.markerpub.publish(marker)


if __name__ == '__main__':
    try:
	relay = Relay()
	relay.spin()
    except rospy.ROSInterruptException:
        pass

