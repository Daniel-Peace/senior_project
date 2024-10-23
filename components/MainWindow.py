from components         import CardWidget, ReportListWidget, ReportWidget, VideoView
from PyQt5.QtGui        import QFont
from PyQt5.QtWidgets    import QHBoxLayout, QMainWindow, QWidget, QVBoxLayout

# This class creates the main window of the application
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



        # -------------------------------------------------------
        # ADD COMPONENTS TO LEFT LAYOUT HERE
        # -------------------------------------------------------
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
        self.infoHboxLayout.addWidget(self.currentDetections)

        # creating and adding card widget to hold currently picked AprilTag
        self.currentlyPickedApriltag = CardWidget('Currently Picked Apriltag', 'Tag ID: --')
        self.infoHboxLayout.addWidget(self.currentlyPickedApriltag)

        # creating and adding card widget to hold the prediction status of each model
        self.modelPredictionStatuses = CardWidget('Model Prediction Status:', '--')
        self.infoHboxLayout.addWidget(self.modelPredictionStatuses)

        # adding bottom infoHboxLayout to leftVboxLayout
        self.mainLeftVboxLayout.addLayout(self.infoHboxLayout)
        # -------------------------------------------------------



        # -------------------------------------------------------
        # ADD COMPONENTS TO RIGHT LAYOUT HERE
        # -------------------------------------------------------
        # creating hbox to hold report widgets
        self.reportHbox = QHBoxLayout()

        # adding list of reports
        self.reportList = ReportListWidget()
        self.reportHbox.addWidget(self.reportList)

        # adding final prediction widget
        self.predictions = ReportWidget(title="Selected Report")
        self.reportHbox.addWidget(self.predictions)

        self.mainRightVboxLayout.addLayout(self.reportHbox)
        # -------------------------------------------------------



        # adding main vertical layouts to main layout
        mainLayout.addLayout(self.mainLeftVboxLayout)
        mainLayout.addLayout(self.mainRightVboxLayout)

        

        