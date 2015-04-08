# Imports
import sys
import rospy
from PyQt4 import QtCore, QtGui, uic
from threading import Thread
from geometry_msgs.msg import Point

# Load GUI
formClass = uic.loadUiType("starGameUI.ui")[0]               

# Ros spin thread
def rosThread():
	rospy.spin()
	
# My QT class
class StarGameWindow(QtGui.QMainWindow, formClass):
    def __init__(self, parent=None):
	# Setup QT gui
        QtGui.QMainWindow.__init__(self, parent)
        self.setupUi(self)

	# Get screen info (resolution)
	self.screen = QtGui.QDesktopWidget().screenGeometry()

	# Load drone image.
	dronePixmap = QtGui.QPixmap('images/drone.png')
	droneScaledPixmap = dronePixmap.scaled(90,90, QtCore.Qt.KeepAspectRatio)
	self.droneLabel.setPixmap(droneScaledPixmap)
	
	# Bind the event handlers
        self.btnExit.clicked.connect(self.btnExitClicked)  

    def btnExitClicked(self):    
	rospy.signal_shutdown("Exit program")
	QtGui.QApplication.quit()

    def moveDrone(self, msg):
	if(msg.x>0 and msg.y>0):
		self.droneLabel.move(msg.x,msg.y)

    def pointCallback(self, msg):
	self.moveDrone(msg)

# Main
if __name__ == "__main__":
	# Load qt gui and stargame window
	app = QtGui.QApplication(sys.argv)
	gameWindow = StarGameWindow(None)

	# Init ROS
	rospy.init_node('starGameNode', anonymous=True)
	rospy.Subscriber("positionPublisher5", Point, gameWindow.pointCallback)

	# Start ros_spin in new thread since it is a blocking call
	threadROS = Thread(target = rosThread)
	threadROS.start()

	# Show game window
	gameWindow.show()
	gameWindow.showFullScreen()
	app.exec_()


