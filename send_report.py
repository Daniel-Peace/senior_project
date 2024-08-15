#!/usr/bin/env python3

# imports
import requests
import rospy
import json
from messages.msg import Critical_report
from messages.msg import Critical_report_response
from messages.msg import Injury_report
from messages.msg import Injury_report_response
from messages.msg import Vitals_report
from messages.msg import Vitals_report_response
from messages.msg import Status
import time

# network constants
TEST_HOST   = 'http://0.0.0.0:80'
HOST        = 'http://10.200.1.100:80'
ENDPOINT_S  = '/api/status'
ENDPOINT_C  = '/api/critical'
ENDPOINT_V  = '/api/vitals'
ENDPOINT_I  = '/api/injury'
ACTIVE_HOST = TEST_HOST

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

# creating ROS topics
status_pub              = rospy.Publisher('status', Status, queue_size=10)
critical_response_pub   = rospy.Publisher('critical_response', Critical_report_response, queue_size=10)
vitals_response_pub     = rospy.Publisher('vitals_response', Vitals_report_response, queue_size=10)
injury_response_pub     = rospy.Publisher('injury_response', Injury_report_response, queue_size=10)

# prints the formatted urls
def print_urls():
    print("\n+-------------------------------------------------------+")
    print("|\t\t\tURLs\t\t\t\t|")
    print("+----------------+--------------------------------------+")
    print("| status url     | " + status_url + "\t|")
    print("+----------------+--------------------------------------+")
    print("| critical url   | " + critical_url + "\t|")
    print("+----------------+--------------------------------------+")
    print("| vitals url     | " + vitals_url + "\t|")
    print("+----------------+--------------------------------------+")
    print("| injury url     | " + injury_url + "\t|")
    print("+----------------+--------------------------------------+")

# prints the formatted http headers
def print_hreaders():
    formatted_json = json.dumps(json_headers, indent = 2)
    print("\n\n---------------------------------------------------------\nraw headers\n---------------------------------------------------------")
    print(json_headers)
    print("---------------------------------------------------------")
    print("\n\n---------------------------------------------------------\nformatted headers\n---------------------------------------------------------")
    print(formatted_json)
    print("---------------------------------------------------------")

# prints a ROS report unformatted
def print_raw_report(r):
    print("\n\n---------------------------------------------------------\nraw report\n---------------------------------------------------------\n")
    print(r)
    print("---------------------------------------------------------")

# prints a ROS report formatted as a JSON
def print_json_report(r):
    print("\n\n---------------------------------------------------------\njson msg\n---------------------------------------------------------")
    print(r)
    print("---------------------------------------------------------")



# posts a critical report
def post_critical_report(r):

    # printing raw report
    print_raw_report(r)
    
    # creating report json
    report = {
        "casualty_id":  r.casualty_id,
        "team":         r.team,
        "system":       r.system,
        "type":         r.type,
        "value":        r.value
    }

    # printing formatted report
    print_json_report(r)

    # sending post request
    response = requests.post(critical_url, headers = json_headers, json = report)

    print(response.json())

    # creating ros msg for response
    critical_report_response = Critical_report_response()

    # initializing message
    critical_report_response.run                  = response.json()["run"]
    critical_report_response.team                 = response.json()["team"]
    critical_report_response.user                 = response.json()["user"]
    critical_report_response.system               = response.json()["system"]
    critical_report_response.clock                = response.json()["clock"]
    critical_report_response.report_id            = response.json()["report_id"]
    critical_report_response.report_timestamp     = response.json()["report_timestamp"]
    critical_report_response.reports_remaining    = response.json()["reports_remaining"]
    critical_report_response.report_status        = response.json()["report_status"]
    critical_report_response.casualty_id          = response.json()["casualty_id"]
    critical_report_response.type                 = response.json()["type"]
    critical_report_response.value                = response.json()["value"]

    # publishing response to injury report
    critical_response_pub.publish(critical_report_response)



# posts a vitals report
def post_vitals_report(r):

    # printing raw report
    print_raw_report(r)
    
    # creating report json
    report = {
        "casualty_id":  r.casualty_id,
        "team":         r.team,
        "system":       r.system,
        "type":         r.type,
        "value":        r.value,
        "time_ago":     r.time_ago
    }

    # printing formatted report
    print_json_report(r)

    # sending post request
    response = requests.post(vitals_url, headers = json_headers, json = report)

    print(response.json())

    # creating ros msg for response
    vitals_report_response = Vitals_report_response()

    # initializing message
    vitals_report_response.run                  = response.json()["run"]
    vitals_report_response.team                 = response.json()["team"]
    vitals_report_response.user                 = response.json()["user"]
    vitals_report_response.system               = response.json()["system"]
    vitals_report_response.clock                = response.json()["clock"]
    vitals_report_response.report_id            = response.json()["report_id"]
    vitals_report_response.report_timestamp     = response.json()["report_timestamp"]
    vitals_report_response.reports_remaining    = response.json()["reports_remaining"]
    vitals_report_response.report_status        = response.json()["report_status"]
    vitals_report_response.casualty_id          = response.json()["casualty_id"]
    vitals_report_response.type                 = response.json()["type"]
    vitals_report_response.value                = int(response.json()["value"])
    vitals_report_response.time_ago             = int(response.json()["time_ago"])

    # publishing response to injury report
    vitals_response_pub.publish(vitals_report_response)



# posts a injury report
def post_injury_report(r):

    # printing raw report
    print_raw_report(r)
    
    # creating report json
    report = {
        "casualty_id":  r.casualty_id,
        "team":         r.team,
        "system":       r.system,
        "type":         r.type,
        "value":        r.value
    }

    # printing formatted report
    print_json_report(r)

    # sending post request
    response = requests.post(injury_url, headers = json_headers, json = report)

    print(response.json())

    # creating ros msg for response
    injury_report_response = Injury_report_response()

    # initializing message
    injury_report_response.run                  = response.json()["run"]
    injury_report_response.team                 = response.json()["team"]
    injury_report_response.user                 = response.json()["user"]
    injury_report_response.system               = response.json()["system"]
    injury_report_response.clock                = response.json()["clock"]
    injury_report_response.report_id            = response.json()["report_id"]
    injury_report_response.report_timestamp     = response.json()["report_timestamp"]
    injury_report_response.reports_remaining    = response.json()["reports_remaining"]
    injury_report_response.report_status        = response.json()["report_status"]
    injury_report_response.casualty_id          = response.json()["casualty_id"]
    injury_report_response.type                 = response.json()["type"]
    injury_report_response.value                = response.json()["value"]

    # publishing response to injury report
    injury_response_pub.publish(injury_report_response)

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
        status_msg = Status()

        # sending get request for status message
        response = requests.get(status_url, headers = json_headers)

        print_json_report(response.json())

        # creating message to publishj
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

        # sleeping to slow the loop down
        time.sleep(5)

# Runs the the following code if this script is run by itself
if __name__ == "__main__":
    print_urls()
    print_hreaders()
    listener()