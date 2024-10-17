from PyQt5.QtWidgets    import QMainWindow, QWidget, QHBoxLayout, QVBoxLayout
from components         import VideoView, CardWidget, LogWidget, ReportWidget, ReportListWidget

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

        # creating main left vertical layout
        self.leftVboxLayout = QVBoxLayout()

        # creating main right vertical layout
        self.rightVboxLayout = QVBoxLayout()

        # -------------------------------------------------------
        # ADD COMPONENTS TO LEFT LAYOUT HERE
        # -------------------------------------------------------
        # creating and adding timer ui
        self.timerHboxLayout = QHBoxLayout()
        self.aprilTagCountdownCard = CardWidget('AprilTag\nCountdown', '--')
        self.timerHboxLayout.addWidget(self.aprilTagCountdownCard)
        self.aprilTagTimerCard = CardWidget('AprilTag\nTimer', '--')
        self.timerHboxLayout.addWidget(self.aprilTagTimerCard)
        self.predictionCountdownCard = CardWidget('Prediction\nCountdown', '--')
        self.timerHboxLayout.addWidget(self.predictionCountdownCard)
        self.predictionTimerCard = CardWidget('Prediction\nTimer', '--')
        self.timerHboxLayout.addWidget(self.predictionTimerCard)
        self.leftVboxLayout.addLayout(self.timerHboxLayout)

        # creating and adding the VideoWidget
        self.videoView = VideoView(self)
        self.leftVboxLayout.addWidget(self.videoView)

        # creating bottom hbox to hold other info cards
        self.infoHboxLayout = QHBoxLayout()

        # creating and adding current AprilTag detections card
        self.currentDetections = CardWidget('Current Apriltag\nDetections', 'id: --')
        self.infoHboxLayout.addWidget(self.currentDetections)

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
        self.predictions = ReportWidget(title="Final Predictions")
        self.rightHbox.addWidget(self.predictions)

        # adding log widget
        self.log = LogWidget()
        self.rightHbox.addWidget(self.log)

        self.rightVboxLayout.addLayout(self.rightHbox)
        # -------------------------------------------------------

        # adding main vertical layouts to main layout
        mainLayout.addLayout(self.leftVboxLayout)
        mainLayout.addLayout(self.rightVboxLayout)

        

        