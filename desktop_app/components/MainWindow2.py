from PyQt5.QtWidgets    import QMainWindow, QWidget, QHBoxLayout, QVBoxLayout
from components         import VideoView, CardWidget

class MainWindow2(QMainWindow):
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
        # =======================================================

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

        # -------------------------------------------------------
        # ADD COMPONENTS TO LEFT LAYOUT HERE
        # -------------------------------------------------------



        # =======================================================

        # adding main vertical layouts to main layout
        mainLayout.addLayout(self.leftVboxLayout)
        mainLayout.addLayout(self.rightVboxLayout)

        

        