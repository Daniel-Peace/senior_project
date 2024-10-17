from PyQt5.QtWidgets    import QWidget, QVBoxLayout, QListWidgetItem
from PyQt5.QtCore       import Qt
from components         import TitleLabel, BodyLabel
from casualty           import Casualty

class ReportWidget(QWidget):
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
        super().__init__(parent)

        # creating and setting main layout for card
        mainLayout = QVBoxLayout()
        self.setLayout(mainLayout)

        # setting style of card
        self.setAttribute(Qt.WA_StyledBackground, True)
        self.setStyleSheet('background-color: #2F343E;''border-radius: 5px;')
        self.setMinimumWidth(250)
        
        # creating and adding title label
        self.titleLabel = TitleLabel(title)
        mainLayout.addWidget(self.titleLabel)

        # creating and adding apriltag label
        self.apriltag = BodyLabel(apriltag)
        mainLayout.addWidget(self.apriltag)
        self.apriltag.setAlignment(Qt.AlignLeft)

        # creating and adding isCoherent label
        self.isCoherent = BodyLabel(isCoherent)
        mainLayout.addWidget(self.isCoherent)
        self.isCoherent.setAlignment(Qt.AlignLeft)

        # creating and adding timeAgo label
        self.timeAgo = BodyLabel(timeAgo)
        mainLayout.addWidget(self.timeAgo)
        self.timeAgo.setAlignment(Qt.AlignLeft)

        # creating and adding severeHemorrhage label
        self.severeHemorrhage = BodyLabel(severeHemorrhage)
        mainLayout.addWidget(self.severeHemorrhage)
        self.severeHemorrhage.setAlignment(Qt.AlignLeft)

        # creating and adding respiratoryDistress label
        self.respiratoryDistress = BodyLabel(respiratoryDistress)
        mainLayout.addWidget(self.respiratoryDistress)
        self.respiratoryDistress.setAlignment(Qt.AlignLeft)

        # creating and adding heartRate label
        self.heartRate = BodyLabel(heartRate)
        mainLayout.addWidget(self.heartRate)
        self.heartRate.setAlignment(Qt.AlignLeft)

        # creating and adding respiratoryRate label
        self.respiratoryRate = BodyLabel(respiratoryRate)
        mainLayout.addWidget(self.respiratoryRate)
        self.respiratoryRate.setAlignment(Qt.AlignLeft)

        # creating and adding traumaTorso label
        self.traumaTorso = BodyLabel(traumaTorso)
        mainLayout.addWidget(self.traumaTorso)
        self.traumaTorso.setAlignment(Qt.AlignLeft)

        # creating and adding traumaHead label
        self.traumaHead = BodyLabel(traumaHead)
        mainLayout.addWidget(self.traumaHead)
        self.traumaHead.setAlignment(Qt.AlignLeft)

        # creating and adding traumaLowerExt label
        self.traumaLowerExt = BodyLabel(traumaLowerExt)
        mainLayout.addWidget(self.traumaLowerExt)
        self.traumaLowerExt.setAlignment(Qt.AlignLeft)

        # creating and adding traumaUpperExt label
        self.traumaUpperExt = BodyLabel(traumaUpperExt)
        mainLayout.addWidget(self.traumaUpperExt)
        self.traumaUpperExt.setAlignment(Qt.AlignLeft)

        # creating and adding alertnessOcular label
        self.alertnessOcular = BodyLabel(alertnessOcular)
        mainLayout.addWidget(self.alertnessOcular)
        self.alertnessOcular.setAlignment(Qt.AlignLeft)

        # creating and adding alertnessVerbal label
        self.alertnessVerbal = BodyLabel(alertnessVerbal)
        mainLayout.addWidget(self.alertnessVerbal)
        self.alertnessVerbal.setAlignment(Qt.AlignLeft)

        # creating and adding alertnessMotor label
        self.alertnessMotor = BodyLabel(alertnessMotor)
        mainLayout.addWidget(self.alertnessMotor)
        self.alertnessMotor.setAlignment(Qt.AlignLeft)

    # updates the title label
    def updateTitle(self, title):
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

    # converts teh QListWidgetItem to a Casualty item and updates the report card
    def updateOnClick(self, item:QListWidgetItem):
        report = item.data(Qt.UserRole)
        self.updateReportValues(report)

    # updates the background color of the card
    def updateBackgroundColor(self, color):
        self.setAttribute(Qt.WA_StyledBackground, True)
        self.setStyleSheet(f"background-color: {color};"'border-radius: 5px;')
    
    # updates the color of all text in the card
    def updateTextColor(self, color):
        self.titleLabel.updateColor(color)
        self.bodyLabel.updateColor(color)