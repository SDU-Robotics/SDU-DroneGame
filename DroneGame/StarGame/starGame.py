# Imports
import sys
import rospy
from PyQt4 import QtCore, QtGui, uic
from threading import Thread
from geometry_msgs.msg import Point
from random import randint

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

	# Variables	
	self.gameRunning = False
	self.score = 5
	self.highscore = 0
	self.updateScore()
	self.xOffset = 800
	self.yOffset = 100	

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

	# Load star image.
	starPixmap = QtGui.QPixmap('images/star.png')
	starScaledPixmap = starPixmap.scaled(60,60, QtCore.Qt.KeepAspectRatio)
	self.labelStar.setPixmap(starScaledPixmap)

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

    def resetGame(self):
	self.score = 0
	self.lcdScore.display(self.score)

    def generateStar(self):
	randX =	randint(self.xOffset,self.xOffset+600)
	randY =	randint(self.yOffset,self.yOffset+600)
	self.labelStar.move(randX,randY)

    def updateScore(self):
	self.score += 1
	self.lcdScore.display(self.score)

    def updateHighscore(self):
	if(self.highscore < self.score):
		self.highscore = self.score
		self.lcdHighscore.display(self.highscore)

    def gameTimerCallback(self):
	#print self.lcdTime.intValue()
	if (self.lcdTime.intValue()-1 < 0):
		self.gameTimer.stop()
		self.updateHighscore()
		self.resetGame()
	else:
		self.lcdTime.display(self.lcdTime.intValue()-1)
		self.updateScore()
		self.generateStar()

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

    def keyPressEvent(self, event):	
	key = event.key()
	if event.isAutoRepeat():
	    	if type(event) == QtGui.QKeyEvent and event.key() == QtCore.Qt.Key_A: 
			print "A"

	    	if type(event) == QtGui.QKeyEvent and event.key() == QtCore.Qt.Key_S:
			print "S"

    def btnExitClicked(self):    
	rospy.signal_shutdown("Exit program")
	QtGui.QApplication.quit()

    def btnResetClicked(self):    
	self.gameTimer.stop()
	self.lcdTime.display(60)

    def btnStartClicked(self):   
	self.resetGame() 
	self.gameTimer.start(1000)

    def moveDrone(self, msg):
	if(msg.x>0 and msg.y>0):
		#self.animateDrone(Point(msg.x,msg.y,0))
		#print "(x,y): ", msg.x, ",", msg.y
		#self.graphicsDronePixmap.setPos(msg.x*2,msg.y*2)
		self.labelDrone.move(self.xOffset + msg.x, self.yOffset + msg.y)

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
	#gameWindow.showFullScreen()
	app.exec_()


