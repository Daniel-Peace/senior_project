from PyQt5.QtWidgets    import QMainWindow, QWidget, QHBoxLayout, QVBoxLayout
from PyQt5.QtGui        import QFont
from components         import VideoView, CardWidget, ReportWidget, ReportListWidget, ApriltagDetectionsWidget

# This class creates the main window of the application
class MainWindow(QMainWindow):
    def __init__(self):
        # initializing parent object
        super().__init__()

        # setting window defaults
        self.setWindowTitle('ROS Monitor')
        self.setGeometry(100, 100, 800, 600)
        self.setStyleSheet("""
            QMainWindow {
                background-color: #282C32;  /* Light gray background */
            }
        """)

        # creating and setting central widget
        centralWidget = QWidget(self)
        self.setCentralWidget(centralWidget)

        # creating main layout
        mainLayout = QHBoxLayout(centralWidget)

        # setting spacing between cards
        mainLayout.setSpacing(10)

        # creating main left vertical layout
        self.leftVboxLayout = QVBoxLayout()

        # creating main right vertical layout
        self.rightVboxLayout = QVBoxLayout()

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
        self.leftVboxLayout.addLayout(self.timerHboxLayout)

        # creating and adding the VideoWidget
        self.videoView = VideoView(self)
        self.leftVboxLayout.addWidget(self.videoView)

        # creating bottom hbox to hold other info cards
        self.infoHboxLayout = QHBoxLayout()

        # creating AprilTag detection widget
        self.currentDetections = ApriltagDetectionsWidget()
        self.infoHboxLayout.addWidget(self.currentDetections)

        # creating and adding current AprilTag detections card
        self.modelPredictionStatuses = CardWidget('Model Prediction Status:', '--')
        # self.modelPredictionStatuses.bodyLabel.setAlignment(Qt.AlignLeft)
        self.infoHboxLayout.addWidget(self.modelPredictionStatuses)


        # adding bottom infoHboxLayout to leftVboxLayout
        self.leftVboxLayout.addLayout(self.infoHboxLayout)
        # -------------------------------------------------------

        # -------------------------------------------------------
        # ADD COMPONENTS TO RIGHT LAYOUT HERE
        # -------------------------------------------------------
        self.rightHbox = QHBoxLayout()

        # adding list of reports
        self.reportList = ReportListWidget()
        self.rightHbox.addWidget(self.reportList)

        # adding final prediction widget
        self.predictions = ReportWidget(title="Selected Report")
        self.rightHbox.addWidget(self.predictions)

        self.rightVboxLayout.addLayout(self.rightHbox)
        # -------------------------------------------------------

        # adding main vertical layouts to main layout
        mainLayout.addLayout(self.leftVboxLayout)
        mainLayout.addLayout(self.rightVboxLayout)

        

        