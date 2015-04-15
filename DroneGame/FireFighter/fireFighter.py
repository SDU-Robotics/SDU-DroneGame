# Imports
import sys
import rospy
from PyQt4 import QtCore, QtGui, uic
from PyQt4.QtGui import *
from PyQt4.QtCore import QObject, pyqtSignal
from threading import Thread
from geometry_msgs.msg import Point
from random import randint
from math import sqrt
# Load GUI
formClass = uic.loadUiType("fireFighterUI.ui")[0]               

# Ros spin thread
def rosThread():
	rospy.spin()
	
class AnimalObj:
	def __init__(self, obj, combinedImg):
		self.obj = obj
		self.combinedImg = combinedImg
		self.rescued = False

	def setRescued(self, rescued):
		self.rescued = rescued

	def isRescued(self):
		return self.rescued
# My QT class
class StarGameWindow(QtGui.QMainWindow, formClass):
    trigger = pyqtSignal(str)

    def __init__(self, parent=None):
	# Setup QT gui
        QtGui.QMainWindow.__init__(self, parent)
        self.setupUi(self)

	self.firstrun = False
	self.isCarryingAnimal = False

	# Variables	
	self.gameRunning = False
	self.score = 5
	self.highscore = 0
	self.updateScore()
	self.xOffset = 800
	self.yOffset = 100
	self.animalLabels = []

	#self.animalLabels.append( AnimalObj( self.labelAnimalPig, "images/droneAnimalPigCombined.png" ) )
	#self.animalLabels.append( AnimalObj( self.labelAnimalPanda, "images/droneAnimalPandaCombined.png" ) )
	self.animalLabels.append( AnimalObj( self.labelAnimalCow, "images/droneAnimalCowCombined.png" ) )
	self.animalLabels.append( AnimalObj( self.labelAnimalTiger, "images/droneAnimalTigerCombined.png" ) )
	#self.animalLabels.append( AnimalObj( self.labelAnimalLion, "images/droneAnimalLionCombined.png" ) )
	self.animalLabels.append( AnimalObj( self.labelAnimalRabbid, "images/droneAnimalRabbidCombined.png" ) )
	#self.animalLabels.append( AnimalObj( self.labelAnimalHippo, "images/droneAnimalHippoCombined.png" ) )

#	self.animalLabels.append(self.labelAnimalPanda)
#	self.animalLabels.append(self.labelAnimalCow)
#	self.animalLabels.append(self.labelAnimalTiger)
#	self.animalLabels.append(self.labelAnimalLion)
#	self.animalLabels.append(self.labelAnimalRabbid)
#	self.animalLabels.append(self.labelAnimalHippo)


	#Setup first animal
#	animalPigPixmap = QtGui.QPixmap('images/animalPig.png')
        #animalPigScaledPixmap = animalPigPixmap.scaled(100,100, QtCore.Qt.KeepAspectRatio)
#	self.labelAnimalPig.setPixmap(animalPigPixmap)

	# Setup logo
	logoPixmap = QtGui.QPixmap('images/sdulogo.png')
	logoScaledPixmap = logoPixmap.scaled(logoPixmap.width()/2,logoPixmap.height()/2, QtCore.Qt.KeepAspectRatio)
	self.labelLogo.setPixmap(logoScaledPixmap)

	# Get screen info (resolution)
	self.screen = QtGui.QDesktopWidget().screenGeometry()

	# Load drone image.
	dronePixmap = QtGui.QPixmap('images/droneHighRes.png')
#	droneScaledPixmap = dronePixmap.scaled(90,90, QtCore.Qt.KeepAspectRatio)
	self.labelDrone.setPixmap(dronePixmap)

	# Load star image.
	#burningHousePixmap = QtGui.QPixmap('images/ringOfFire.png')
#	#burningHouseScaledPixmap = burningHousePixmap.scaled(400,470, QtCore.Qt.KeepAspectRatio)
	#self.labelBurningHouse.setPixmap(burningHousePixmap)


        # Load the file into a QMovie
        self.movie = QMovie("images/bonfire.gif")
 
        #size = self.movie.scaledSize()
        #self.setGeometry(200, 200, size.width(), size.height())
 
        #self.movie_screen = QLabel()
        # Make label fit the gif
        #self.labelLogo.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Expanding)
        #self.labelLogo.setAlignment(QtCore.Qt.AlignCenter)
 
        # Create the layout
        #main_layout = QVBoxLayout()
        #main_layout.addWidget(self.movie_screen)
 
        #self.setLayout(main_layout)
 
        # Add the QMovie object to the label
        self.movie.setCacheMode(QMovie.CacheAll)
        #self.movie.setSpeed(100)
        self.labelFire1.setMovie(self.movie)
	self.labelFire2.setMovie(self.movie)
	self.labelFire3.setMovie(self.movie)
	self.labelFire4.setMovie(self.movie)
	self.labelFire5.setMovie(self.movie)
	self.labelFire6.setMovie(self.movie)

        self.movie.start()





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


	self.trigger.connect(self.handle_trigger)
    def handle_trigger(self, string):
	self.labelDrone.setPixmap(QtGui.QPixmap(string))

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
		transfx = msg.y
		transfy = 300 - msg.x

		dronex = transfx*3 + 670
		droney = transfy*3 + 60

		droneWindowx = transfx*3
		droneWindowy = transfy*3

		self.labelDrone.move(dronex,droney)
		if self.isCarryingAnimal == False:
			for animal in self.animalLabels:
				if not animal.isRescued():
					dist = sqrt((droneWindowx - animal.obj.x())**2 + (droneWindowy - animal.obj.y())**2)
					if dist < 30:
						animal.obj.setVisible(False)
						self.trigger.emit(animal.combinedImg)
						animal.setRescued(True)
						self.isCarryingAnimal = True
		else:
			if dronex > 1245 and droney < 220:
				self.isCarryingAnimal = False
				print "Dropping of animal"
				self.trigger.emit("images/droneHighRes.png")
				self.updateScore()

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


