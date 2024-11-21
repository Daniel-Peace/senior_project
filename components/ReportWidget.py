# -------------------------------------------------------------------------------------------
# Daniel Peace
# CSUCI / Coordinated Robotics - DTC
# -------------------------------------------------------------------------------------------
# This class creates a widget that can hold and display all information from a report
#
# -------------------------------------------------------------------------------------------

from casualty           import Casualty
from components         import BodyLabel, TitleLabel
from PyQt5.QtCore       import Qt
from PyQt5.QtWidgets    import QListWidgetItem, QSizePolicy, QWidget, QVBoxLayout

# this class creates a card that displays all values of a report on a casualty
class ReportWidget(QWidget):
    # constructor
    def __init__(
            self, 
            title, 
            apriltag='AprilTag: --', 
            isCoherent='Is Coherent: --', 
            timeAgo='Time Ago: --', 
            severeHemorrhage='Severe Hemorrhage: --', 
            respiratoryDistress='Respiratory Distress: --', 
            heartRate='Heart Rate: --', 
            respiratoryRate='Respiratory Rate: --', 
            traumaHead='Trauma Head: --', 
            traumaTorso='Trauma Torso: --', 
            traumaLowerExt='Trauma Lower Ext: --',
            traumaUpperExt='Trauma Upper Ext: --',
            alertnessOcular='Alertness Ocular: --',
            alertnessVerbal='Alertness Verbal: --',
            alertnessMotor='Alertness Motor: --',
            parent = None
            ):
        
        # instantiate parent object
        super().__init__(parent)

        # creating and setting main layout for card
        mainLayout = QVBoxLayout()
        self.setLayout(mainLayout)

        # setting style of card
        self.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Expanding)
        self.setAttribute(Qt.WA_StyledBackground, True)
        self.setStyleSheet('background-color: #2F343E;''border-radius: 15px;')
        
        # creating and adding title label
        self.titleLabel = TitleLabel(title)
        self.titleLabel.setMaximumHeight(60)
        self.titleLabel.setMinimumHeight(60)
        mainLayout.addWidget(self.titleLabel)
    
        # creating sub-widget to contain body labels for report values
        self.valueWidget = QWidget()
        self.valueWidget.setStyleSheet(f"color: #ADB2BD;background-color: #42474f;padding: 10px;border-radius: 15px;")
        
        # creating and setting main layout for sub-widget
        self.valueLayout = QVBoxLayout()
        self.valueWidget.setLayout(self.valueLayout)

        # creating and adding apriltag label
        self.apriltag = BodyLabel(apriltag)
        self.valueLayout.addWidget(self.apriltag)
        self.apriltag.setAlignment(Qt.AlignLeft)

        # creating and adding isCoherent label
        self.isCoherent = BodyLabel(isCoherent)
        self.valueLayout.addWidget(self.isCoherent)
        self.isCoherent.setAlignment(Qt.AlignLeft)

        # creating and adding timeAgo label
        self.timeAgo = BodyLabel(timeAgo)
        self.valueLayout.addWidget(self.timeAgo)
        self.timeAgo.setAlignment(Qt.AlignLeft)

        # creating and adding severeHemorrhage label
        self.severeHemorrhage = BodyLabel(severeHemorrhage)
        self.valueLayout.addWidget(self.severeHemorrhage)
        self.severeHemorrhage.setAlignment(Qt.AlignLeft)

        # creating and adding respiratoryDistress label
        self.respiratoryDistress = BodyLabel(respiratoryDistress)
        self.valueLayout.addWidget(self.respiratoryDistress)
        self.respiratoryDistress.setAlignment(Qt.AlignLeft)

        # creating and adding heartRate label
        self.heartRate = BodyLabel(heartRate)
        self.valueLayout.addWidget(self.heartRate)
        self.heartRate.setAlignment(Qt.AlignLeft)

        # creating and adding respiratoryRate label
        self.respiratoryRate = BodyLabel(respiratoryRate)
        self.valueLayout.addWidget(self.respiratoryRate)
        self.respiratoryRate.setAlignment(Qt.AlignLeft)

        # creating and adding traumaTorso label
        self.traumaTorso = BodyLabel(traumaTorso)
        self.valueLayout.addWidget(self.traumaTorso)
        self.traumaTorso.setAlignment(Qt.AlignLeft)

        # creating and adding traumaHead label
        self.traumaHead = BodyLabel(traumaHead)
        self.valueLayout.addWidget(self.traumaHead)
        self.traumaHead.setAlignment(Qt.AlignLeft)

        # creating and adding traumaLowerExt label
        self.traumaLowerExt = BodyLabel(traumaLowerExt)
        self.valueLayout.addWidget(self.traumaLowerExt)
        self.traumaLowerExt.setAlignment(Qt.AlignLeft)

        # creating and adding traumaUpperExt label
        self.traumaUpperExt = BodyLabel(traumaUpperExt)
        self.valueLayout.addWidget(self.traumaUpperExt)
        self.traumaUpperExt.setAlignment(Qt.AlignLeft)

        # creating and adding alertnessOcular label
        self.alertnessOcular = BodyLabel(alertnessOcular)
        self.valueLayout.addWidget(self.alertnessOcular)
        self.alertnessOcular.setAlignment(Qt.AlignLeft)

        # creating and adding alertnessVerbal label
        self.alertnessVerbal = BodyLabel(alertnessVerbal)
        self.valueLayout.addWidget(self.alertnessVerbal)
        self.alertnessVerbal.setAlignment(Qt.AlignLeft)

        # creating and adding alertnessMotor label
        self.alertnessMotor = BodyLabel(alertnessMotor)
        self.valueLayout.addWidget(self.alertnessMotor)
        self.alertnessMotor.setAlignment(Qt.AlignLeft)

        # adding sub-widget to main widget
        mainLayout.addWidget(self.valueWidget)

    # updates the title label
    def updateTitleText(self, title):
        self.titleLabel.updateText(title)

    # updates the values in the report card
    def updateReportValues(self, casualty:Casualty):
        self.apriltag.updateText("AprilTag: " + str(casualty.apriltag))
        self.isCoherent.updateText("Is Coherent: " + str(casualty.is_coherent))
        self.timeAgo.updateText("Time Ago: " + str(casualty.time_ago))
        self.severeHemorrhage.updateText("Severe Hemorrhage: " + str(casualty.severe_hemorrhage))
        self.respiratoryDistress.updateText("Respiratory Distress: " + str(casualty.respiratory_distress))
        self.heartRate.updateText("Heart Rate: " + str(casualty.heart_rate))
        self.respiratoryRate.updateText("Respiratory Rate: " + str(casualty.respiratory_rate))
        self.traumaHead.updateText("Trauma Head: " + str(casualty.trauma_head))
        self.traumaTorso.updateText("Trauma Torso: " + str(casualty.trauma_torso))
        self.traumaLowerExt.updateText("Trauma Lower Ext: " + str(casualty.trauma_lower_ext))
        self.traumaUpperExt.updateText("Trauma Upper Ext: " + str(casualty.trauma_upper_ext))
        self.alertnessOcular.updateText("Alertness Ocular: " + str(casualty.alertness_ocular))
        self.alertnessVerbal.updateText("Alertness Verbal: " + str(casualty.alertness_verbal))
        self.alertnessMotor.updateText("Alertness Motor: " + str(casualty.alertness_motor))

    # converts the QListWidgetItem to a Casualty item and updates the report card
    def updateOnClick(self, item:QListWidgetItem):
        report = item.data(Qt.UserRole)
        self.updateReportValues(report)

    # updates the background color of the card
    def updateBackgroundColor(self, color):
        self.setAttribute(Qt.WA_StyledBackground, True)
        self.setStyleSheet(f"background-color: {color};"'border-radius: 15px;')