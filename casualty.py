# -------------------------------------------------------------------------------------------
# Daniel Peace
# CSUCI / Coordinated Robotics - DTC
# -------------------------------------------------------------------------------------------
# This class provides members for storing all data partaining to a casualty. It also
# provides several methods for working on an instance of this class. You can change
# which system and team name is used with the reports by changing the constants TEAM_NAME
# and SYSTEM at the top of the program.
#
# METHOD NAME: reset
# DESCRIPTION: This function resets an instance of this class to the default values used
# by the contructor.
#
# METHOD NAME: publish_reports
# DESCRIPTION: This method calls three other methods:
#   - publish_critical_reports
#   - publish_vitals_reprots
#   - publish_injury_reports
#
# METHOD NAME: publish_critical_reports
# DESCRIPTION: This method takes self and creates all relevent critical reports from the data
# stored in its members. It publishes these reports to the "critical_report" topic
#
# METHOD NAME: publish_vitals_reports
# DESCRIPTION: This method takes self and creates all relevent vitals reports from the data
# stored in its members. It publishes these reports to the "vitals_report" topic
#
# METHOD NAME: publish_injury_reports
# DESCRIPTION: This method takes self and creates all relevent injury reports from the data
# stored in its members. It publishes these reports to the "injury_report" topic
#
# METHOD NAME: print_self
# DESCRIPTION: This method formats and prints all members of this class.
# -------------------------------------------------------------------------------------------

import rospy

# ROS messages
from messages.msg import Critical_report
from messages.msg import Vitals_report
from messages.msg import Injury_report

# creating publishers
c_publisher = rospy.Publisher('critical_report', Critical_report, queue_size=10)
v_publisher = rospy.Publisher('vitals_report', Vitals_report, queue_size=10)
i_publisher = rospy.Publisher('injury_report', Injury_report, queue_size=10)

class Casualty:
   
    # ================================== CHANGE THESE IF NEEDED ===================================

    TEAM_NAME   = "coordinated robotics"
    SYSTEM      = "JOE"

    # =============================================================================================

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

    # This array contains all "types" of afflications (used for creation of reports)
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
            is_coherent             = True,
            time_ago                = -1,
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
        self.time_ago               = time_ago

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

    # resets class object
    def reset(self):
        self.apriltag               = -1
        self.is_coherent            = True
        self.time_ago               = -1
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
        report              = Critical_report()
        report.casualty_id  = self.apriltag
        report.team         = self.TEAM_NAME
        report.system       = self.SYSTEM
        report.type         = self.affliction_types_strings[self.SEVERE_HEMORRHAGE]
        report.value        = self.severe_hemorrhage
        print(report)
        print("------------------------------------------------------")
        print("\u001b[34m[-] \u001b[0mPublishing critical report")
        print("------------------------------------------------------")
        c_publisher.publish(report)
        report.type         = self.affliction_types_strings[self.RESPIRATORY_DISTRESS]
        report.value        = self.respiratory_distress
        print(report)
        print("------------------------------------------------------")
        print("\u001b[34m[-] \u001b[0mPublishing critical report")
        print("------------------------------------------------------")
        c_publisher.publish(report)

    # publishes vitals reports
    def publish_vitals_reprots(self):
        report              = Vitals_report()
        report.casualty_id  = self.apriltag
        report.team         = self.TEAM_NAME
        report.system       = self.SYSTEM
        report.time_ago     = self.time_ago
        report.type         = self.affliction_types_strings[self.HEART_RATE]
        report.value        = self.heart_rate
        print(report)
        print("------------------------------------------------------")
        print("\u001b[34m[-] \u001b[0mPublishing vitals report")
        print("------------------------------------------------------")
        v_publisher.publish(report)
        report.type         = self.affliction_types_strings[self.RESPIRATORY_RATE]
        report.value        = self.respiratory_rate
        print(report)
        print("------------------------------------------------------")
        print("\u001b[34m[-] \u001b[0mPublishing vitals report")
        print("------------------------------------------------------")
        v_publisher.publish(report)

    # publishes injury reports
    def publish_injury_reports(self):
        report              = Injury_report()
        report.casualty_id  = self.apriltag
        report.team         = self.TEAM_NAME
        report.system       = self.SYSTEM
        report.type         = self.affliction_types_strings[self.TRAUMA_HEAD]
        report.value        = self.trauma_head
        print(report)
        print("------------------------------------------------------")
        print("\u001b[34m[-] \u001b[0mPublishing injury report")
        print("------------------------------------------------------")
        i_publisher.publish(report)
        report.type         = self.affliction_types_strings[self.TRAUMA_TORSO]
        report.value        = self.trauma_torso
        print(report)
        print("------------------------------------------------------")
        print("\u001b[34m[-] \u001b[0mPublishing injury report")
        print("------------------------------------------------------")
        i_publisher.publish(report)
        report.type         = self.affliction_types_strings[self.TRAUMA_LOWER_EXT]
        report.value        = self.trauma_lower_ext
        print(report)
        print("------------------------------------------------------")
        print("\u001b[34m[-] \u001b[0mPublishing injury report")
        print("------------------------------------------------------")
        i_publisher.publish(report)
        report.type         = self.affliction_types_strings[self.TRAUMA_UPPER_EXT]
        report.value        = self.trauma_upper_ext
        print(report)
        print("------------------------------------------------------")
        print("\u001b[34m[-] \u001b[0mPublishing injury report")
        print("------------------------------------------------------")
        i_publisher.publish(report)
        report.type         = self.affliction_types_strings[self.ALERTNESS_OCULAR]
        report.value        = self.alertness_ocular
        print(report)
        print("------------------------------------------------------")
        print("\u001b[34m[-] \u001b[0mPublishing injury report")
        print("------------------------------------------------------")
        i_publisher.publish(report)
        report.type         = self.affliction_types_strings[self.ALERTNESS_VERBAL]
        report.value        = self.alertness_verbal
        print(report)
        print("------------------------------------------------------")
        print("\u001b[34m[-] \u001b[0mPublishing injury report")
        print("------------------------------------------------------")
        i_publisher.publish(report)
        report.type         = self.affliction_types_strings[self.ALERTNESS_MOTOR]
        report.value        = self.alertness_motor
        print(report)
        print("------------------------------------------------------")
        print("\u001b[34m[-] \u001b[0mPublishing injury report")
        print("------------------------------------------------------")
        i_publisher.publish(report)

    # prints the members of this class
    def print_self(self):
        print("AprilTag:\t\t\t" + str(self.apriltag))
        print("Is coherent:\t\t\t" + str(self.apriltag))
        print("Time Ago:\t\t\t" + str(self.time_ago))
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
