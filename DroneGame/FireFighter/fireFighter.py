# Imports
import sys
import rospy
from PyQt4 import QtCore, QtGui, uic
from threading import Thread
from geometry_msgs.msg import Point

# Load GUI
formClass = uic.loadUiType("fireFighterUI.ui")[0]

# Ros spin thread
def rosThread():
	rospy.spin()
	
# My QT class
class StarGameWindow(QtGui.QMainWindow, formClass):
    def __init__(self, parent=None):
	# Setup QT gui
        QtGui.QMainWindow.__init__(self, parent)
        self.setupUi(self)

	# Setup logo
	logoPixmap = QtGui.QPixmap('images/sdulogo.png')
	logoScaledPixmap = logoPixmap.scaled(logoPixmap.width()/2,logoPixmap.height()/2, QtCore.Qt.KeepAspectRatio)
	self.labelLogo.setPixmap(logoScaledPixmap)

	# Get screen info (resolution)
	self.screen = QtGui.QDesktopWidget().screenGeometry()

	# Load drone image.
	dronePixmap = QtGui.QPixmap('images/droneHighRes.png')
	droneScaledPixmap = dronePixmap.scaled(90,90, QtCore.Qt.KeepAspectRatio)
	self.labelDrone.setPixmap(droneScaledPixmap)

	# Graphics scene stuff
	'''scene = QtGui.QGraphicsScene()
	scene.setSceneRect(0, 0, 600, 600);
	self.graphicsView.setScene(scene)
	self.graphicsDronePixmap = scene.addPixmap(droneScaledPixmap)'''

	# Timer stuff	
	self.animationTimer = QtCore.QTimeLine(1000/30)
	#self.animationTimer.setFrameRange(0, 1000)
	self.gameTimer = QtCore.QTimer()
	self.gameTimer.timeout.connect(self.gameTimerCallback)
	
	# Old drone pos
	self.oldDronePos = Point(0,0,0)	

	# Bind the event handlers
        self.btnExit.clicked.connect(self.btnExitClicked)  
        self.btnReset.clicked.connect(self.btnResetClicked)  
        self.btnStart.clicked.connect(self.btnStartClicked)  

    def gameTimerCallback(self):
	#print self.lcdTime.intValue()
	if (self.lcdTime.intValue()-1 < 0):
		self.gameTimer.stop()
	else:
		self.lcdTime.display(self.lcdTime.intValue()-1)

    def animateDrone(self, newPos):
	self.animationTimer.stop()
	animation = QtGui.QGraphicsItemAnimation()
	animation.setItem(self.graphicsDronePixmap)
	animation.setTimeLine(self.animationTimer)

	deltaPoint = Point(0,0,0)
	deltaPoint.x = newPos.x - self.oldDronePos.x
	deltaPoint.y = newPos.y - self.oldDronePos.y

	interval = 10
	for i in range (0,interval):
		tempPos = Point(0,0,0)
		tempPos.x = self.oldDronePos.x + deltaPoint.x * i/interval
		tempPos.y = self.oldDronePos.y + deltaPoint.y * i/interval
		animation.setPosAt(i/interval, QtCore.QPointF(tempPos.x, tempPos.y))

	self.oldDronePos = newPos;
	self.animationTimer.start()

    def btnExitClicked(self):    
	rospy.signal_shutdown("Exit program")
	QtGui.QApplication.quit()

    def btnResetClicked(self):    
	self.gameTimer.stop()
	self.lcdTime.display(60)

    def btnStartClicked(self):    
	self.gameTimer.start(1000)

    def moveDrone(self, msg):
	if(msg.x>0 and msg.y>0):
		#self.animateDrone(Point(msg.x,msg.y,0))
		#print "(x,y): ", msg.x, ",", msg.y
		#self.graphicsDronePixmap.setPos(msg.x*2,msg.y*2)
		self.labelDrone.move(800+msg.x,100+msg.y)

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
#	gameWindow.showFullScreen()
	app.exec_()


