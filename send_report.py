# -------------------------------------------------------------------------------------------
# Daniel Peace
# CSUCI / Coordinated Robotics - DTC
# -------------------------------------------------------------------------------------------
# This program is responsible for submitting finalized predictions to the scoring
# server. It creates callback functions for each report type and submits
# all ROS messages received from those topics. It also peridically sends a request
# for a status update. The responses from each HTTP request is published to a corresponding
# ROS topic:
#   - critical_response
#   - vitals_response
#   - injury_response
#   - status
# -------------------------------------------------------------------------------------------

import json
import requests
import rospy
import time
from messages.msg       import Critical_report
from messages.msg       import Critical_report_response
from messages.msg       import Injury_report
from messages.msg       import Injury_report_response
from messages.msg       import Status
from messages.msg       import Vitals_report
from messages.msg       import Vitals_report_response

# general constants
BAR = "---------------------------------------------------------------------"

# network constants
TEST_HOST   = 'http://0.0.0.0:80'
HOST        = 'http://10.200.1.100:80'
ACTIVE_HOST = TEST_HOST
ENDPOINT_S  = '/api/status'
ENDPOINT_C  = '/api/critical'
ENDPOINT_V  = '/api/vitals'
ENDPOINT_I  = '/api/injury'

# bearer tokens
TEST_TOKEN      = 'Bearer eyJhbGciOiJIUzI1NiIsInR5cCI6IkpXVCJ9.eyJzdWIiOiI4M2Q3OGM4ZS04MzhhLTQ0NzctOWM3Yi02N2VmMTZlNWY3MTYiLCJpIjowfQ.i4KuwEtc5_6oIYz5TDWcdzl5bMkvCpLZTSZG2Avy84w'
ACTIVE_TOKEN    = TEST_TOKEN

# creating URLs
status_url      = "{}{}".format(ACTIVE_HOST, ENDPOINT_S)
critical_url    = "{}{}".format(ACTIVE_HOST, ENDPOINT_C)
vitals_url      = "{}{}".format(ACTIVE_HOST, ENDPOINT_V)
injury_url      = "{}{}".format(ACTIVE_HOST, ENDPOINT_I)

# headers for http requests
json_headers = {
    "accept" : "application/json",
    "Authorization" : ACTIVE_TOKEN,
    "Content-Type" : "application/json"
    }

# creating publishers
status_pub              = rospy.Publisher('status', Status, queue_size=10)
critical_response_pub   = rospy.Publisher('critical_response', Critical_report_response, queue_size=10)
vitals_response_pub     = rospy.Publisher('vitals_response', Vitals_report_response, queue_size=10)
injury_response_pub     = rospy.Publisher('injury_response', Injury_report_response, queue_size=10)

# used for printing system messages
def system_print(s):
    print("\u001b[34m[-] \u001b[0m" + s)

# prints the formatted urls
def print_urls():
    print("\n+-------------------------------------------------------+")
    print("|\t\t\tURLs\t\t\t\t|")
    print("+----------------+--------------------------------------+")
    print("| status url     | " + status_url + "\t\t|")
    print("+----------------+--------------------------------------+")
    print("| critical url   | " + critical_url + "\t|")
    print("+----------------+--------------------------------------+")
    print("| vitals url     | " + vitals_url + "\t\t|")
    print("+----------------+--------------------------------------+")
    print("| injury url     | " + injury_url + "\t\t|")
    print("+----------------+--------------------------------------+")

# prints the formatted http headers
def print_hreaders():
    print("\n" + BAR)
    system_print("Formatted headers")
    print(BAR)
    print(json.dumps(json_headers, indent = 2))
    print(BAR)

# posts a critical report to the scoring server
def post_critical_report(r):
    if r.casualty_id < 0:
        system_print("No casualty ID found. Skipping report")
        return

    # creating report json
    report = {
        "casualty_id":  r.casualty_id,
        "team":         r.team,
        "system":       r.system,
        "type":         r.type,
        "value":        r.value
    }

    # printing report
    system_print("Critical report:\n" + BAR + "\n" + str(json.dumps(report, indent=2)) + "\n" + BAR)

    try:
        # sending post request
        response = requests.post(critical_url, headers = json_headers, json = report)
        response.raise_for_status()

        # printing response
        system_print("Critical report response:\n" + BAR + "\n" + str(json.dumps(response.json(), indent=2)) + "\n" + BAR)

        # creating ROS msg for response
        critical_report_response = Critical_report_response()

        # initializing message
        critical_report_response.run                  = response.json()["run"]
        critical_report_response.team                 = response.json()["team"]
        critical_report_response.user                 = response.json()["user"]
        critical_report_response.system               = response.json()["system"]
        critical_report_response.clock                = response.json()["clock"]
        critical_report_response.report_id            = response.json()["report_id"]
        critical_report_response.report_timestamp     = response.json()["report_timestamp"]
        critical_report_response.reports_remaining    = int(response.json()["reports_remaining"])
        critical_report_response.report_status        = response.json()["report_status"]
        critical_report_response.casualty_id          = int(response.json()["casualty_id"])
        critical_report_response.type                 = response.json()["type"]
        critical_report_response.value                = int(response.json()["value"])

        # publishing response to injury report
        critical_response_pub.publish(critical_report_response)
                
    except requests.exceptions.RequestException as errex: 
        system_print("Exception request")
        print(BAR + "\n")
        print(errex)
        print(BAR)
    

# posts a vitals report
def post_vitals_report(r):
    if r.casualty_id < 0:
        system_print("No casualty ID found. Skipping report")
        return

    # creating report json
    report = {
        "casualty_id":  r.casualty_id,
        "team":         r.team,
        "system":       r.system,
        "type":         r.type,
        "value":        r.value,
        "time_ago":     r.time_ago
    }

    # printing report
    system_print("Vitals report:\n" + BAR + "\n" + str(json.dumps(report, indent=2)) + "\n" + BAR)

    try:
        # sending post request
        response = requests.post(vitals_url, headers = json_headers, json = report)
        response.raise_for_status()

        # printing response
        system_print("Vitals report response:\n" + BAR + "\n" + str(json.dumps(response.json(), indent=2)) + "\n" + BAR)

        # creating ROS msg for response
        vitals_report_response = Vitals_report_response()

        # initializing message
        vitals_report_response.run                  = response.json()["run"]
        vitals_report_response.team                 = response.json()["team"]
        vitals_report_response.user                 = response.json()["user"]
        vitals_report_response.system               = response.json()["system"]
        vitals_report_response.clock                = response.json()["clock"]
        vitals_report_response.report_id            = response.json()["report_id"]
        vitals_report_response.report_timestamp     = response.json()["report_timestamp"]
        vitals_report_response.reports_remaining    = int(response.json()["reports_remaining"])
        vitals_report_response.report_status        = response.json()["report_status"]
        vitals_report_response.casualty_id          = int(response.json()["casualty_id"])
        vitals_report_response.type                 = response.json()["type"]
        vitals_report_response.value                = int(response.json()["value"])
        vitals_report_response.time_ago             = int(response.json()["time_ago"])

        # publishing response to injury report
        vitals_response_pub.publish(vitals_report_response)
       
    except requests.exceptions.RequestException as errex: 
        system_print("Exception request")
        print(BAR + "\n")
        print(errex)
        print(BAR)

# posts a injury report
def post_injury_report(r):
    if r.casualty_id < 0:
        system_print("No casualty ID found. Skipping report")
        return

    # creating report json
    report = {
        "casualty_id":  r.casualty_id,
        "team":         r.team,
        "system":       r.system,
        "type":         r.type,
        "value":        r.value
    }

    # printing report
    system_print("Injury report:\n" + BAR + "\n" + str(json.dumps(report, indent=2)) + "\n" + BAR)

    try:
        # sending post request
        response = requests.post(injury_url, headers = json_headers, json = report)
        response.raise_for_status()

        # printing response
        system_print("Injury report response:\n" + BAR + "\n" + str(json.dumps(response.json(), indent=2)) + "\n" + BAR)

        # creating ROS msg for response
        injury_report_response = Injury_report_response()

        # initializing message
        injury_report_response.run                  = response.json()["run"]
        injury_report_response.team                 = response.json()["team"]
        injury_report_response.user                 = response.json()["user"]
        injury_report_response.system               = response.json()["system"]
        injury_report_response.clock                = response.json()["clock"]
        injury_report_response.report_id            = response.json()["report_id"]
        injury_report_response.report_timestamp     = response.json()["report_timestamp"]
        injury_report_response.reports_remaining    = int(response.json()["reports_remaining"])
        injury_report_response.report_status        = response.json()["report_status"]
        injury_report_response.casualty_id          = int(response.json()["casualty_id"])
        injury_report_response.type                 = response.json()["type"]
        injury_report_response.value                = int(response.json()["value"])

        # publishing response to injury report
        injury_response_pub.publish(injury_report_response)
       
    except requests.exceptions.RequestException as errex: 
        system_print("Exception request")
        print(BAR + "\n")
        print(errex)
        print(BAR)

#listens for status messages and registers a callback function for the injury
def listener():
    # initializing node
    rospy.init_node('report_node', anonymous=True)

    # registering callback functions
    rospy.Subscriber('injury_report', Injury_report, post_injury_report)
    rospy.Subscriber('critical_report', Critical_report, post_critical_report)
    rospy.Subscriber('vitals_report', Vitals_report, post_vitals_report)

    # looping until program is quit
    while not rospy.is_shutdown():


        # sending get request for status message
        try:
            response = requests.get(status_url, headers = json_headers)
            response.raise_for_status()

            # printing response from server
            system_print("Status response:\n" + BAR + "\n" + str(json.dumps(response.json(), indent=2)) + "\n" + BAR)

            # creating message to publish
            status_msg = Status()
            status_msg.clock                                        = response.json()["clock"]
            status_msg.team                                         = response.json()["team"]
            status_msg.user                                         = response.json()["user"]
            status_msg.remaining_reports.critical.hemorrhage        = response.json()["remaining_reports"]["critical"]["hemorrhage"]
            status_msg.remaining_reports.critical.distress          = response.json()["remaining_reports"]["critical"]["distress"]
            status_msg.remaining_reports.vitals.heart               = response.json()["remaining_reports"]["vitals"]["heart"]
            status_msg.remaining_reports.vitals.respiratory         = response.json()["remaining_reports"]["vitals"]["respiratory"]
            status_msg.remaining_reports.injury.trauma_head         = response.json()["remaining_reports"]["injury"]["trauma_head"]
            status_msg.remaining_reports.injury.trauma_torso        = response.json()["remaining_reports"]["injury"]["trauma_torso"]
            status_msg.remaining_reports.injury.trauma_lower_ext    = response.json()["remaining_reports"]["injury"]["trauma_lower_ext"]
            status_msg.remaining_reports.injury.trauma_upper_ext    = response.json()["remaining_reports"]["injury"]["trauma_upper_ext"]
            status_msg.remaining_reports.injury.alertness_ocular    = response.json()["remaining_reports"]["injury"]["alertness_ocular"]
            status_msg.remaining_reports.injury.alertness_verbal    = response.json()["remaining_reports"]["injury"]["alertness_verbal"]
            status_msg.remaining_reports.injury.alertness_motor     = response.json()["remaining_reports"]["injury"]["alertness_motor"]

            # publishing status message
            status_pub.publish(status_msg)
        except requests.exceptions.RequestException as errex: 
            system_print("Exception request")
            print(BAR + "\n")
            print(errex)
            print(BAR + "\n")
            

        # sleeping to slow the loop down
        time.sleep(5)

# Runs the the following code if this script is run by itself
if __name__ == "__main__":
    print_urls()
    print_hreaders()
    listener()
