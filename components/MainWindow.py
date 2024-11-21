# -------------------------------------------------------------------------------------------
# Daniel Peace
# CSUCI / Coordinated Robotics - DTC
# -------------------------------------------------------------------------------------------
# This is the main window that holds all other widgets and layouts. There are some banners
# informing you wher you should add components. At this point, however, the UI has been
# finalized for a demo and may need further tweaking to acheive the desired look if you
# add new components.
#
# -------------------------------------------------------------------------------------------

from components         import CardWidget, ReportListWidget, ReportWidget, VideoView
from PyQt5.QtGui        import QFont
from PyQt5.QtWidgets    import QHBoxLayout, QMainWindow, QWidget, QVBoxLayout
from PyQt5.QtCore       import Qt, QObject, QEvent

# this class creates the main window of the application
class MainWindow(QMainWindow):
    def __init__(self):
        # initializing parent object
        super().__init__()

        # setting window defaults
        self.setWindowTitle('ROS Monitor')
        self.setGeometry(100, 100, 800, 600)
        self.setStyleSheet('background-color: #282C32;')

        # creating and setting central widget
        centralWidget = QWidget(self)
        self.setCentralWidget(centralWidget)

        # creating main layout
        mainLayout = QHBoxLayout(centralWidget)
        mainLayout.setSpacing(10)
        centralWidget.setLayout(mainLayout)

        # creating main left vertical layout
        self.mainLeftVboxLayout = QVBoxLayout()

        # creating main right vertical layout
        self.mainRightVboxLayout = QVBoxLayout()



        # =============================================================================================
        # ADD COMPONENTS TO LEFT LAYOUT HERE
        # =============================================================================================
        # creating and adding timer ui
        self.timerHboxLayout = QHBoxLayout()
        self.aprilTagCountdownCard = CardWidget('AprilTag Countdown', '--')
        self.aprilTagCountdownCard.bodyLabel.setFont(QFont("Arial", 75))
        self.timerHboxLayout.addWidget(self.aprilTagCountdownCard)
        self.aprilTagTimerCard = CardWidget('AprilTag Timer', '--')
        self.aprilTagTimerCard.bodyLabel.setFont(QFont("Arial", 75))
        self.timerHboxLayout.addWidget(self.aprilTagTimerCard)
        self.predictionCountdownCard = CardWidget('Prediction Countdown', '--')
        self.predictionCountdownCard.bodyLabel.setFont(QFont("Arial", 75))
        self.timerHboxLayout.addWidget(self.predictionCountdownCard)
        self.predictionTimerCard = CardWidget('Prediction Timer', '--')
        self.predictionTimerCard.bodyLabel.setFont(QFont("Arial", 75))
        self.timerHboxLayout.addWidget(self.predictionTimerCard)
        self.mainLeftVboxLayout.addLayout(self.timerHboxLayout)

        # creating and adding the VideoWidget
        self.videoView = VideoView(self)
        self.mainLeftVboxLayout.addWidget(self.videoView)

        # creating info hbox
        self.infoHboxLayout = QHBoxLayout()

        # creating and adding card widget to hold current AprilTag detections
        self.currentDetections = CardWidget('Current AprilTag Detections', 'Tag ID: --')
        self.currentDetections.setMaximumHeight(300)
        self.infoHboxLayout.addWidget(self.currentDetections)

        # creating and adding card widget to hold currently picked AprilTag
        self.currentlyPickedApriltag = CardWidget('Currently Picked Apriltag', 'Tag ID: --')
        self.currentlyPickedApriltag.setMaximumHeight(300)
        self.infoHboxLayout.addWidget(self.currentlyPickedApriltag)

        # creating and adding card widget to hold the prediction status of each model
        self.modelPredictionStatuses = CardWidget('Model Prediction Status:', '--')
        self.modelPredictionStatuses.setMaximumHeight(300)
        self.infoHboxLayout.addWidget(self.modelPredictionStatuses)

        # adding bottom infoHboxLayout to leftVboxLayout
        self.mainLeftVboxLayout.addLayout(self.infoHboxLayout)
        # =============================================================================================

        # =============================================================================================
        # ADD COMPONENTS TO RIGHT LAYOUT HERE
        # =============================================================================================
        # creating hbox to hold report widgets
        self.reportHbox = QHBoxLayout()

        # adding list of reports
        self.reportList = ReportListWidget()
        self.reportHbox.addWidget(self.reportList)

        # adding final prediction widget
        self.predictions = ReportWidget(title="Selected Report")
        self.reportHbox.addWidget(self.predictions)

        # adding reportHbox to main right widget
        self.mainRightVboxLayout.addLayout(self.reportHbox)

        # creating and adding loop status card
        self.loopState = CardWidget('Loop State', 'Waiting to assign AprilTag')
        self.loopState.setMinimumHeight(300)
        self.mainRightVboxLayout.addWidget(self.loopState)
        # =============================================================================================

        # adding main vertical layouts to main layout
        mainLayout.addLayout(self.mainLeftVboxLayout)
        mainLayout.addLayout(self.mainRightVboxLayout)

        # install an event filter on the main window to capture mouse events
        self.installEventFilter(self)

    # used for detecting when the mouse clicks outside of the list widget
    def eventFilter(self, source, event):
        # check if the event is a mouse press
        if event.type() == QEvent.MouseButtonPress:
            # get the position of the click
            click_pos = event.pos()
            
            # check if the click is outside the target widget but inside the main window
            if source == self and not self.reportList.list.geometry().contains(click_pos):
                self.reportList.list.clearSelection()
                
        
        # call the base class eventFilter method to continue processing
        return super().eventFilter(source, event)    

        