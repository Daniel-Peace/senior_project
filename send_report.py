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
TEST_HOST   = 'http://192.168.1.1:80'
HOST        = 'http://10.200.1.100:80'
ENDPOINT_S  = '/api/status'
ENDPOINT_C  = '/api/critical'
ENDPOINT_V  = '/api/vitals'
ENDPOINT_I  = '/api/injury'
ACTIVE_HOST = HOST

# creating URLs
status_url      = "{}{}".format(ACTIVE_HOST, ENDPOINT_S)
critical_url    = "{}{}".format(ACTIVE_HOST, ENDPOINT_C)
vitals_url      = "{}{}".format(ACTIVE_HOST, ENDPOINT_V)
injury_url      = "{}{}".format(ACTIVE_HOST, ENDPOINT_I)

# headers for http requests
json_headers = {
    "accept" : "application/json",
    "Authorization" : "Bearer eyJhbGciOiJIUzI1NiIsInR5cCI6IkpXVCJ9.eyJzdWIiOiI0ZjQwZmE1ZC00ODBiLTQ1NGMtYTU1ZS1hYTkwOTc3MjM0YjEiLCJpIjowfQ.APMBNs1LE4Ngvq9W4QgfzZeggKBX9VwuWpst_jLMG74",
    "Content-Type" : "application/json"
    }

# creating ROS topics
status_pub              = rospy.Publisher('status', Status, queue_size=10)
critical_response_pub   = rospy.Publisher('critical_response', Critical_report_response, queue_size=10)
vitals_response_pub     = rospy.Publisher('vitals_response', Vitals_report_response, queue_size=10)
injury_response_pub     = rospy.Publisher('injury_response', Injury_report_response, queue_size=10)

# prints the formatted urls for validation
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

# prints the formatted http headers for validation
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

    # # sending post request
    # response = requests.post(critical_url, headers = json_headers, json = report)

    # print(response.json())

    # # creating ros msg for response
    # report_response = Critical_report_response()

    # # initializing message
    # report_response.run                  = response.run
    # report_response.team                 = response.team
    # report_response.user                 = response.user
    # report_response.system               = response.system
    # report_response.clock                = response.clock
    # report_response.report_id            = response.report_id
    # report_response.report_timestamp     = response.report_timestamp
    # report_response.reports_remaining    = response.reports_remaining
    # report_response.report_status        = response.report_status
    # report_response.casualty_id          = response.casualty_id
    # report_response.type                 = response.type
    # report_response.value                = response.value

    # # publishing response to injury report
    # critical_response_pub.publish(report_response)

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

    # # sending post request
    # response = requests.post(vitals_url, headers = json_headers, json = report)

    # print(response.json())

    # # creating ros msg for response
    # report_response = Vitals_report_response()

    # # initializing message
    # report_response.run                  = response.run
    # report_response.team                 = response.team
    # report_response.user                 = response.user
    # report_response.system               = response.system
    # report_response.clock                = response.clock
    # report_response.report_id            = response.report_id
    # report_response.report_timestamp     = response.report_timestamp
    # report_response.reports_remaining    = response.reports_remaining
    # report_response.report_status        = response.report_status
    # report_response.casualty_id          = response.casualty_id
    # report_response.type                 = response.type
    # report_response.value                = response.value
    # report_response.time_ago             = response.time_ago

    # # publishing response to injury report
    # vitals_response_pub.publish(report_response)

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

    # # sending post request
    # response = requests.post(injury_url, headers = json_headers, json = report)

    # print(response.json())

    # # creating ros msg for response
    # injury_report_response = Injury_report_response()

    # # initializing message
    # injury_report_response.run                  = response.run
    # injury_report_response.team                 = response.team
    # injury_report_response.user                 = response.user
    # injury_report_response.system               = response.system
    # injury_report_response.clock                = response.clock
    # injury_report_response.report_id            = response.report_id
    # injury_report_response.report_timestamp     = response.report_timestamp
    # injury_report_response.reports_remaining    = response.reports_remaining
    # injury_report_response.report_status        = response.report_status
    # injury_report_response.casualty_id          = response.casualty_id
    # injury_report_response.type                 = response.type
    # injury_report_response.value                = response.value

    # # publishing response to injury report
    # injury_response_pub.publish(injury_report_response)

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

        # # sending get request for status message
        # response = requests.get(status_url, headers = json_headers)

        # # creating message to publishj
        # status_msg.clock                                        = response.clock
        # status_msg.team                                         = response.team
        # status_msg.user                                         = response.user
        # status_msg.remaining_reports.critical.hemorrhage        = response.remaining_reports.critical.hemorrhage
        # status_msg.remaining_reports.critical.distress          = response.remaining_reports.critical.distress
        # status_msg.remaining_reports.vitals.heart               = response.remaining_reports.vitals.heart
        # status_msg.remaining_reports.vitals.respiratory         = response.remaining_reports.vitals.respiratory
        # status_msg.remaining_reports.injury.trauma_head         = response.remaining_reports.injury.trauma_head
        # status_msg.remaining_reports.injury.trauma_torso        = response.remaining_reports.injury.trauma_torso
        # status_msg.remaining_reports.injury.trauma_lower_ext    = response.remaining_reports.injury.trauma_lower_ext
        # status_msg.remaining_reports.injury.trauma_upper_ext    = response.remaining_reports.injury.trauma_upper_ext
        # status_msg.remaining_reports.injury.alertness_ocular    = response.remaining_reports.injury.alertness_ocular
        # status_msg.remaining_reports.injury.alertness_verbal    = response.remaining_reports.injury.alertness_verbal
        # status_msg.remaining_reports.injury.alertness_motor     = response.remaining_reports.injury.alertness_motor

        # # publishing status message
        # status_pub.publish(status_msg)

        # sleeping to slow the loop down
        time.sleep(0.2)

# Runs the the following code if this script is run by itself
if __name__ == "__main__":
    print_urls()
    print_hreaders()
    listener()