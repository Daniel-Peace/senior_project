# imports
import rospy
import time
import json

from casualty import Casualty_types as CT
from casualty import Casualtys_status

from messages.msg import Critical_report
from messages.msg import Injury_report
from messages.msg import Prediction
from messages.msg import Vitals_report
from messages.msg import Timer_status

def handle_timer_status(timer_status):
    my_casualty = Casualtys_status()
    print(json.dumps(my_casualty.__dict__))

    my_casualty_2 = Casualtys_status(april_tag=2,severe_hemorrhage=1)
    print(json.dumps(my_casualty_2.__dict__))

    if timer_status.timer_status == True:
        print("start predicting")
    else:
        print("submit reports")


if __name__ == "__main__":
    # initializing node
    rospy.init_node('vote_and_create', anonymous=True)

    # registering callback functions
    rospy.Subscriber('prediction_timer_status', Timer_status, handle_timer_status)

    while not rospy.is_shutdown():
        # sleeping to slow the loop down
        time.sleep(0.2)