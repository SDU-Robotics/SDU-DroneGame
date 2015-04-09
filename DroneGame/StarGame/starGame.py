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
	self.score = 0
	self.highscore = 0
	self.updateScore()
	self.xOffset = 610
	self.yOffset = 0
	self.xSize = 600
	self.ySize = 600	
	self.currentPos = Point(0,0,0)
	self.oldPos = Point(1920/2,1080/2,0)
	self.starPos = Point(0,0,0)	

	# Setup logo
	logoPixmap = QtGui.QPixmap('images/sdulogo.png')
	logoScaledPixmap = logoPixmap.scaled(logoPixmap.width()/2,logoPixmap.height()/2, QtCore.Qt.KeepAspectRatio)
	self.labelLogo.setPixmap(logoScaledPixmap)

	# Get screen info (resolution)
	self.screen = QtGui.QDesktopWidget().screenGeometry()

	# Load star image.
	starPixmap = QtGui.QPixmap('images/star.png')
	starScaledPixmap = starPixmap.scaled(60,60, QtCore.Qt.KeepAspectRatio)
	self.labelStar.setPixmap(starScaledPixmap)
	self.labelStar.move(1920/2,1080/4)
	self.labelStar.setVisible(False)

	# Load drone image.
	dronePixmap = QtGui.QPixmap('images/droneHighResRotated.png')
	droneScaledPixmap = dronePixmap.scaled(90,90, QtCore.Qt.KeepAspectRatio)
	self.labelDrone.setPixmap(droneScaledPixmap)
	self.labelDrone.move(1920/2,1080/2)

	# Graphics scene stuff
	'''scene = QtGui.QGraphicsScene()
	scene.setSceneRect(0, 0, self.xSize, self.ySize);
	self.graphicsView.setScene(scene)
	self.graphicsDronePixmap = scene.addPixmap(droneScaledPixmap)'''

	# Timer stuff	
	self.animationTimer = QtCore.QTimeLine(1000/30)
	#self.animationTimer.setFrameRange(0, 1000)
	self.gameTimer = QtCore.QTimer()
	self.gameTimer.timeout.connect(self.gameTimerCallback)
	self.starTimer = QtCore.QTimer()
	self.starTimer.timeout.connect(self.starTimerCallback)

	# Bind the event handlers
        self.btnExit.clicked.connect(self.btnExitClicked)  
        self.btnReset.clicked.connect(self.btnResetClicked)  
        self.btnStart.clicked.connect(self.btnStartClicked)  

    def generateStar(self):
	self.starPos.x = randint(self.xOffset,self.xOffset+self.xSize)
	self.starPos.y = randint(self.yOffset,self.yOffset+self.ySize)
	self.labelStar.move(self.starPos.x,self.starPos.y)

    def updateScore(self):
	self.score += 1
	self.lcdScore.display(self.score)

    def updateHighscore(self):
	if(self.highscore < self.score):
		self.highscore = self.score
		self.lcdHighscore.display(self.highscore)

    def starTimerCallback(self):
	if(abs(self.starPos.x-self.currentPos.x-600)<50 and abs(self.starPos.y-self.currentPos.y)<50):
		self.updateScore()
		self.generateStar()
	

    def gameTimerCallback(self):
	if(self.lcdTime.intValue()-1 < 0):
		self.updateHighscore()
		self.resetGame()
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

    def keyPressEvent(self, event):
	# Manual control
	if(self.gameRunning == True):
		movePixel = 20	
		key = event.key()
	    	if key == QtCore.Qt.Key_W:  
			self.currentPos.y = self.oldPos.y - movePixel
		if key == QtCore.Qt.Key_D:  
			self.currentPos.x = self.oldPos.x + movePixel
		if key == QtCore.Qt.Key_A:  
			self.currentPos.x = self.oldPos.x - movePixel
		if key == QtCore.Qt.Key_S:  
			self.currentPos.y = self.oldPos.y + movePixel
		
		if(self.currentPos.y>=0 and self.currentPos.x>=0):
			self.oldPos = self.currentPos
			self.moveDrone(self.currentPos)

    def btnExitClicked(self):    
	rospy.signal_shutdown("Exit program")
	QtGui.QApplication.quit()

    def btnResetClicked(self):  
	self.resetGame()

    def resetGame(self):
	self.starTimer.stop()
	self.gameTimer.stop()
	self.labelStar.setVisible(False)
	self.gameRunning = False  

    def btnStartClicked(self):  
	self.score = 0
	self.lcdScore.display(self.score) 
	self.lcdTime.display(60)
	self.generateStar()
	self.labelStar.setVisible(True)
	self.gameRunning = True
	self.gameTimer.start(1000)
	self.starTimer.start(100)

    def moveDrone(self, msg):
	if(msg.x>0 and msg.y>0):
		#self.animateDrone(Point(msg.x,msg.y,0))
		#print "(x,y): ", msg.x, ",", msg.y
		#self.graphicsDronePixmap.setPos(msg.x*2,msg.y*2)
		self.labelDrone.move(self.xOffset + msg.x, self.yOffset + msg.y)

	#self.oldPos = self.currentPos 

    def pointCallback(self, msg):
	self.moveDrone(msg)

# Main
if __name__ == "__main__":
	# Load qt gui and stargame window
	app = QtGui.QApplication(sys.argv)
	gameWindow = StarGameWindow(None)

	# Init ROS
	rospy.init_node('starGameNode', anonymous=True)
	#rospy.Subscriber("positionPublisher5", Point, gameWindow.pointCallback)

	# Start ros_spin in new thread since it is a blocking call
	threadROS = Thread(target = rosThread)
	threadROS.start()

	# Show game window
	gameWindow.show()
	gameWindow.showFullScreen()
	app.exec_()


