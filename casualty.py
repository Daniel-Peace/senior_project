import rospy
from messages.msg import Critical_report
from messages.msg import Vitals_report
from messages.msg import Injury_report

class Casualty:
    # constants
    TEAM_NAME   = "coordinated robotics"
    SYSTEM      = "JOE"

    # critical afflication options
    SEVERE_HEMORRHAGE       = 0
    RESPIRATORY_DISTRESS    = 1

    # vitals afflication options
    HEART_RATE              = 2
    RESPIRATORY_RATE        = 3

    # injury afflication options
    TRAUMA_HEAD             = 4
    TRAUMA_TORSO            = 5
    TRAUMA_LOWER_EXT        = 6
    TRAUMA_UPPER_EXT        = 7
    ALERTNESS_OCULAR        = 8
    ALERTNESS_VERBAL        = 9
    ALERTNESS_MOTOR         = 10

    affliction_types_strings = [
        "severe_hemorrhage",
        "respiratory_distress",
        "hr",
        "rr",
        "trauma_head",
        "trauma_torso",
        "trauma_lower_ext",
        "trauma_upper_ext",
        "alertness_ocular",
        "alertness_verbal",
        "alertness_motor"]

    # constructor
    def __init__(
            self, april_tag         = -1, 
            is_coherent             = -1, 
            severe_hemorrhage       = -1, 
            respiratory_distress    = -1, 
            heart_rate              = -1, 
            respiratory_rate        = -1, 
            trauma_head             = -1, 
            trauma_torso            = -1, 
            trauma_lower_ext        = -1, 
            trauma_upper_ext        = -1, 
            alertness_ocular        = -1, 
            alertness_verbal        = -1, 
            alertness_motor         = -1) -> None:
        
        self.apriltag               = april_tag
        self.is_coherent            = is_coherent

        # critical
        self.severe_hemorrhage      = severe_hemorrhage
        self.respiratory_distress   = respiratory_distress

        # vitals
        self.heart_rate             = heart_rate
        self.respiratory_rate       = respiratory_rate

        # injury
        self.trauma_head            = trauma_head
        self.trauma_torso           = trauma_torso
        self.trauma_lower_ext       = trauma_lower_ext
        self.trauma_upper_ext       = trauma_upper_ext
        self.alertness_ocular       = alertness_ocular
        self.alertness_verbal       = alertness_verbal
        self.alertness_motor        = alertness_motor

    # resets all members' values to -1
    def reset(self):
        self.apriltag               = -1
        self.is_coherent            = -1
        self.severe_hemorrhage      = -1
        self.respiratory_distress   = -1
        self.heart_rate             = -1
        self.respiratory_rate       = -1
        self.trauma_head            = -1
        self.trauma_torso           = -1
        self.trauma_lower_ext       = -1
        self.trauma_upper_ext       = -1
        self.alertness_ocular       = -1
        self.alertness_verbal       = -1
        self.alertness_motor        = -1

    # publishers all members to respective topics
    def publish_reports(self):
        self.publish_critical_reports()
        self.publish_vitals_reprots()
        self.publish_injury_reports()

    # publishes critical reports
    def publish_critical_reports(self):
        publisher           = rospy.Publisher('critical_report', Critical_report, queue_size=10)
        report              = Critical_report()
        report.casualty_id  = self.apriltag
        report.team         = self.TEAM_NAME
        report.system       = self.SYSTEM
        report.type         = self.affliction_types_strings[self.SEVERE_HEMORRHAGE]
        report.value        = self.severe_hemorrhage
        print(report)
        print("------------------------------------------------------")
        publisher.publish(report)
        report.type         = self.affliction_types_strings[self.RESPIRATORY_DISTRESS]
        report.value        = self.respiratory_distress
        print(report)
        print("------------------------------------------------------")
        publisher.publish(report)

    # publishes vitals reports
    def publish_vitals_reprots(self):
        publisher           = rospy.Publisher('vitals_report', Vitals_report, queue_size=10)
        report              = Vitals_report()
        report.casualty_id  = self.apriltag
        report.team         = self.TEAM_NAME
        report.system       = self.SYSTEM
        report.type         = self.affliction_types_strings[self.HEART_RATE]
        report.value        = self.heart_rate
        print(report)
        print("------------------------------------------------------")
        publisher.publish(report)
        report.type         = self.affliction_types_strings[self.RESPIRATORY_RATE]
        report.value        = self.respiratory_rate
        print(report)
        print("------------------------------------------------------")
        publisher.publish(report)

    # publishes injury reports
    def publish_injury_reports(self):
        publisher           = rospy.Publisher('injury_report', Injury_report, queue_size=10)
        report              = Injury_report()
        report.casualty_id  = self.apriltag
        report.team         = self.TEAM_NAME
        report.system       = self.SYSTEM
        report.type         = self.affliction_types_strings[self.TRAUMA_HEAD]
        report.value        = self.trauma_head
        print(report)
        print("------------------------------------------------------")
        publisher.publish(report)
        report.type         = self.affliction_types_strings[self.TRAUMA_TORSO]
        report.value        = self.trauma_torso
        print(report)
        print("------------------------------------------------------")
        publisher.publish(report)
        report.type         = self.affliction_types_strings[self.TRAUMA_LOWER_EXT]
        report.value        = self.trauma_lower_ext
        print(report)
        print("------------------------------------------------------")
        publisher.publish(report)
        report.type         = self.affliction_types_strings[self.TRAUMA_UPPER_EXT]
        report.value        = self.trauma_upper_ext
        print(report)
        print("------------------------------------------------------")
        publisher.publish(report)
        report.type         = self.affliction_types_strings[self.ALERTNESS_OCULAR]
        report.value        = self.alertness_ocular
        print(report)
        print("------------------------------------------------------")
        publisher.publish(report)
        report.type         = self.affliction_types_strings[self.ALERTNESS_VERBAL]
        report.value        = self.alertness_verbal
        print(report)
        print("------------------------------------------------------")
        publisher.publish(report)
        report.type         = self.affliction_types_strings[self.ALERTNESS_MOTOR]
        report.value        = self.alertness_motor
        print(report)
        print("------------------------------------------------------")
        publisher.publish(report)

    # prints the members of this class
    def print_self(self):
        print("AprilTag:\t\t\t" + str(self.apriltag))
        print("Is coherent:\t\t\t" + str(self.apriltag))
        print("Severe Hemorrhage:\t\t" + str(self.severe_hemorrhage))
        print("Respiratory Distress:\t\t" + str(self.respiratory_distress))
        print("Heart Rate:\t\t\t" + str(self.heart_rate))
        print("Respiratory Rate:\t\t" + str(self.respiratory_rate))
        print("Trauma Head:\t\t\t" + str(self.trauma_head))
        print("Trauma Torso:\t\t\t" + str(self.trauma_torso))
        print("Trauma Lower Ext.:\t\t" + str(self.trauma_lower_ext))
        print("Trauma Upper Ext.:\t\t" + str(self.trauma_upper_ext))
        print("Alertness Ocular:\t\t" + str(self.alertness_ocular))
        print("Alertness Verbal:\t\t" + str(self.alertness_verbal))
        print("Alertness Motor:\t\t" + str(self.alertness_motor))
