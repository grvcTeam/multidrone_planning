#!/usr/bin/env python
from multidrone_msgs.msg import SASRole
from multidrone_msgs.srv import DirectorEvent, DirectorEventRequest, SelectRole, SelectRoleRequest, SendXML, SendXMLRequest, ValidateMission, ValidateMissionRequest, Clear, ClearRequest
from std_srvs.srv import Trigger, TriggerRequest
import rospkg
import rospy
import json
from geometry_msgs.msg import PointStamped
import time
from os import system
from os import listdir
from os.path import isfile, join
import signal
import sys


def main_menu():
    
    print "\nMain menu of the pydashboard. Please choose the option to send:"
    print "1. Event enrolment"
    print "2. Mission enrolment"
    print "3. Validate mission"
    print "4. Select role"
    print "5. Safe to go service"
    print "6. Director event"
    print "7. Car speed 1 m/s"
    print "8. Abort service"
    print "9. Clear missions and events"
    print "10. Car speed 2 m/s"

    selected = raw_input(" >> ")
    system("clear")
    if selected == "1":
        event_enrolment_menu()
    elif selected == "2":
        mission_enrolment_menu()
    elif selected == "3":
        validate_mission_menu()
    elif selected == "4":
        select_role_menu()
    elif selected == "5":
        ready_to_go_service()
    elif selected == "6":
        director_event_menu()
    elif selected == "7":
        car_speed_menu()
    elif selected == "8":
        abort_menu()
    elif selected == "9":
        clear_menu()
    elif selected == "10":
        car_speed_2_menu()

    else:
        system("clear")
        print "Not a valid option."


# 1. Event enrolment:
def event_enrolment_menu():
    #show the files in the directory:
    onlyfiles = [f for f in listdir(event_path) if isfile(join(event_path, f))]
    print "Event enrolment menu. The next events' XML are available:"
    cont = 0
    orded_list = sorted(onlyfiles)
    for i in range(len(orded_list)):
        num = i+int(1)
        print("%d. %s" %(num, orded_list[i]) )
        cont = cont+1
    print("Press any other key to quit.")
    selected = raw_input(" >> ")
    try:
        if not( selected.isalpha() ) and int(selected)>=1 and int(selected) <= len(orded_list): # selected.isalpha() return true if selected is a leter
            send_event_xml(orded_list[int(selected)-1])
            main_menu()
        else:
            system("clear")
            print "Not a valid option, returning to main menu."
            main_menu()
    except:
        system("clear")
        print "Not a valid option, returning to main menu."
        main_menu()

def send_event_xml(xml_event_name):
    system("clear")
    print("%s selected" %xml_event_name )
    xml = open( event_path + xml_event_name, 'r').read()
    xml_event = SendXMLRequest()
    xml_event.xml = xml
    try:
        print event_enrolment_client.call(xml_event)
    except rospy.ServiceException, e:
        print "\nService call failed: %s"%e


# 2. Mission enrolment:
def mission_enrolment_menu():
    #show the files in the directory:
    onlyfiles = [f for f in listdir(mission_path) if isfile(join(mission_path, f))]
    print "Mission enrolment menu. The next missions' XML are available:"
    cont = 0
    orded_list = sorted(onlyfiles)
    for i in range(len(orded_list)):
        num = i+int(1)
        print("%d. %s" %(num, orded_list[i]) )
        cont = cont+1
    print("Press any other key to quit.")
    selected = raw_input(" >> ")
    try:
        if not( selected.isalpha() ) and int(selected)>=1 and int(selected) <= len(orded_list): # selected.isalpha() return true if selected is a leter
            send_mission_xml(orded_list[int(selected)-1])
            main_menu()
        else:
            system("clear")
            print "Not a valid option, returning to main menu."
            main_menu()
    except:
        system("clear")
        print "Not a valid option, returning to main menu."
        main_menu()

def send_mission_xml(xml_mission_name):
    system("clear")
    print("%s selected" %xml_mission_name )
    xml = open( mission_path + xml_mission_name, 'r').read()
    xml_mission = SendXMLRequest()
    xml_mission.xml = xml
    try:
        print mission_enrolment_client.call(xml_mission)
    except rospy.ServiceException, e:
        print "\nService call failed: %s"%e


# 3. Validate mission:
def validate_mission_menu():
    system("clear")
    print "Validation request selected."
    validation_request = ValidateMissionRequest()
    # validation_request.stem_event_id = ""   # TODO: fill stem_event_id?
    try:
        print validate_mission_client.call(validation_request)
    except rospy.ServiceException, e:
        print "\nService call failed: %s"%e


# 4. Select role:
def select_role_menu():
    system("clear")
    print "Select role request selected."
    select_role_request = SelectRoleRequest()
    select_role_request.mission_role = raw_input("Write mission_role >> ")
    keep_in_loop = True
    while keep_in_loop and not rospy.is_shutdown():
        sas_roles_struct = SASRole()
        sas_roles_struct.mission_id = raw_input("Write mission_id >> ")
        sas_roles_struct.sas_role = raw_input("Write sas_role >> ")
        select_role_request.sas_roles.append(sas_roles_struct)
        print "Keep inserting new pair of mission_id-sas_role? y: yes. Any other key: no."
        selected = raw_input(" >> ")
        if selected == "y":
            keep_in_loop = True
        else:
            keep_in_loop = False
    try:
        print select_role_client.call(select_role_request)
    except rospy.ServiceException, e:
        print "\nService call failed: %s"%e


# 5. Safe to go service
def ready_to_go_service():
    system("clear")
    print "How many drones do you want to set safe to go?"
    selected = raw_input("Write 1, 2 or 3 >> ")
    req = TriggerRequest()
    if selected == "1":
        try:
            safe_to_go_client = rospy.ServiceProxy('/drone_1/safe_to_go',Trigger)
            safe_to_go_client.call()
        except rospy.ServiceException, e:
            print "\nService call failed: %s"%e
    elif selected == "2":
        try:
            safe_to_go_client = rospy.ServiceProxy('/drone_1/safe_to_go',Trigger)
            safe_to_go_client.call(req)
        except rospy.ServiceException, e:
            print "\nService call failed: %s"%e
        try:
            safe_to_go_client_2 = rospy.ServiceProxy('/drone_2/safe_to_go',Trigger)
            safe_to_go_client_2.call(req)
        except rospy.ServiceException, e:
            print "\nService call failed: %s"%e
    elif selected == "3":
        try:
            safe_to_go_client = rospy.ServiceProxy('/drone_1/safe_to_go',Trigger)
            safe_to_go_client.call(req)
        except rospy.ServiceException, e:
            print "\nService call failed: %s"%e
        try:
            safe_to_go_client_2 = rospy.ServiceProxy('/drone_2/safe_to_go',Trigger)
            safe_to_go_client_2.call(req)
        except rospy.ServiceException, e:
            print "\nService call failed: %s"%e
        try:
            safe_to_go_client_3 = rospy.ServiceProxy('/drone_3/safe_to_go',Trigger)
            safe_to_go_client_3.call(req)
        except rospy.ServiceException, e:
            print "\nService call failed: %s"%e
    else:
        system("clear")
        print "Not a valid option, returning to main menu."
        main_menu()


# 6. Director event:
def director_event_menu():
    print "Event menu. Select the event:"
    print "1. GET_READY"
    print "2. START_RACE"
    print "3. START_CYCLING_RACE"
    print "4. SECOND_EVENT"
    print "5. a79b2ad5-f420-4512-97a7-75fcc7da1898"
    print "6. bb1377f1-edc2-45c7-a06a-51d14daaf6bf"
    print "7. svq_kar_35e0dabc-c7c6-44fa-8f94-756cac78dcdf"
    print "Press any other key to quit."
    selected = raw_input(" >> ")
    if selected == "1":
        event_id = "GET_READY"
        send_director_event(event_id)
    elif selected == "2":
        event_id = "START_RACE"
        send_director_event(event_id)
    elif selected == "3":
        event_id = "START_CYCLING_RACE"
        send_director_event(event_id)
    elif selected == "4":
        event_id = "SECOND_EVENT"
        send_director_event(event_id)
    elif selected == "5":
        event_id = "a79b2ad5-f420-4512-97a7-75fcc7da1898"
        send_director_event(event_id)
    elif selected == "6":
        event_id = "bb1377f1-edc2-45c7-a06a-51d14daaf6bf"
        send_director_event(event_id)
    elif selected == "7":
        event_id = "35e0dabc-c7c6-44fa-8f94-756cac78dcdf"
        send_director_event(event_id)
    else:
        system("clear")
        print "Not a valid option, returning to main menu."
        main_menu()

def send_director_event(event_id):
    try:
        system("clear")
        print event_id + " selected."
        event = DirectorEventRequest()
        event.event_id = event_id
        print director_event_client.call(event)
    except rospy.ServiceException, e:
        print "\nService call failed: %s"%e


# 7. Car speed 1 m/s:
def car_speed_menu():
    system("rostopic pub --once /drc_vehicle_xp900/speed/cmd std_msgs/Float64 \"data: 1.0\"")
    system("clear")
    print "Car moving at 1m/s"


# 8. Abort:
def abort_menu():
    try:
        print abort_service.call()
    except rospy.ServiceException, e:
        print "\nService call failed: %s"%e


# 9. Clear missions and events
def clear_menu():
    try:
        system("clear")
        print "Clear missions and events selected."
        print "To delete an specific mission or event write its UUID. Write nothing to clear all."
        clear_request = ClearRequest()
        clear_request.uuid = raw_input("UUID >> ")
        print clear_client.call(clear_request)
    except rospy.ServiceException, e:
        print "\nService call failed: %s"%e


# 10. Car speed 2 m/s:
def car_speed_2_menu():
    system("rostopic pub --once /drc_vehicle_xp900/speed/cmd std_msgs/Float64 \"data: 2\"")
    system("clear")
    print "Car moving at 2m/s"


# Finish the execution directly when Ctrl+C is pressed (signal.SIGINT received), without escalating to SIGTERM.
def signal_handler(sig, frame):
    print('Ctrl+C pressed, signal.SIGINT received.')
    sys.exit(0)


if __name__ == "__main__":
    rospy.init_node('pydashboard', anonymous=True)
    event_path = rospy.get_param('~xml_path') + "/events/"
    mission_path = rospy.get_param('~xml_path') + "/missions/"
    event_enrolment_client = rospy.ServiceProxy('mission_controller/send_event_xml',SendXML)
    abort_service = rospy.ServiceProxy('/mission_controller/abort',Trigger)
    mission_enrolment_client = rospy.ServiceProxy('mission_controller/send_mission_xml',SendXML)
    validate_mission_client = rospy.ServiceProxy('mission_controller/validate',ValidateMission)
    select_role_client = rospy.ServiceProxy('mission_controller/select_role',SelectRole)
    clear_client = rospy.ServiceProxy('mission_controller/clear',Clear)
    director_event_client = rospy.ServiceProxy('mission_controller/director_event',DirectorEvent)
    signal.signal(signal.SIGINT, signal_handler)    # Associate signal SIGINT (Ctrl+C pressed) to handler (function "signal_handler")

    system("clear")
    print "Welcome to the pydashboard,"

    while not rospy.is_shutdown():
        main_menu()
        time.sleep(0.1)
