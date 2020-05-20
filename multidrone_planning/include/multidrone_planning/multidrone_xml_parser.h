/**
 * MULTIDRONE Project:
 *
 * Multidrone XML parser.
 * 
 * Made using the following libraries (all with MIT Licenses):
 *  - "pugixml" library, version 1.8, has been used to parse the XML string.
 *  - "nlohmann/json" library, version 3.1.2, has been used to parse json strings inside the XML.
 *  - "HowardHinnant/date" library has been used used to extract time information from date strings.
 * 
 * MIT License 
 * 
 * Copyright of the library "pugixml" (c) 2006-2017 Arseny Kapoulkine
 * 
 * Copyright of the library "nlohmann/json" (c) 2013-2018 Niels Lohmann
 * 
 * Copyright of the library "HowardHinnant/date":
 *  Copyright (c) 2015, 2016, 2017 Howard Hinnant
 *  Copyright (c) 2016 Adrian Colomitchi
 *  Copyright (c) 2017 Florian Dang
 *  Copyright (c) 2017 Paul Thompson
 *  Copyright (c) 2018 Tomasz Kamiński
 * 
 * Permission is hereby granted, free of charge, to any person
 * obtaining a copy of this software and associated documentation
 * files (the "Software"), to deal in the Software without
 * restriction, including without limitation the rights to use,
 * copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the
 * Software is furnished to do so, subject to the following
 * conditions:
 * 
 * The above copyright notice and this permission notice shall be
 * included in all copies or substantial portions of the Software.
 * 
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND,
 * EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES
 * OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND
 * NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT
 * HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY,
 * WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
 * FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR
 * OTHER DEALINGS IN THE SOFTWARE.
 * 
 */

#ifndef XML_PARSER_H
#define XML_PARSER_H

#include <string>
#include <vector>
#include <map>
#include <iostream>
#include <iomanip>
#include <geometry_msgs/PointStamped.h>

#include <multidrone_msgs/ShootingAction.h>
#include <multidrone_msgs/ShootingActionError.h>
#include <multidrone_msgs/ShootingRole.h>
#include <multidrone_msgs/ShootingType.h>
#include <multidrone_msgs/TargetType.h>
#include "json.hpp"
#include "date.h"
#include "multidrone_kml_parser/pugixml.hpp"
#include "multidrone_kml_parser/geographic_to_cartesian.hpp"

namespace multidrone { namespace XMLparser {

// #define DEBUG_XML_PARSER

using json = nlohmann::json;
using namespace date;


// Inline function that do the XML parse job for the version of the XML.
inline EventStruct event_XML_parser(const std::string& _XML_string, const std::map<std::string,EventStruct>& _events, std::string& _time_zone, std::string& _stem_event_id, const std::string& _absolute_origin_of_time_string) {

    EventStruct event_to_return;
    EventStruct empty_event_to_return_if_error;

    try {

        std::string::size_type sz;  // Used for std::stod for the string to number conversion.

        std::string format_xd_dateTime = "%Y-%m-%dT%TZ";            // This format (xs:dateTime): 2018-12-17T09:31:47Z

        sys_seconds absolute_origin_of_time;
        std::istringstream absolute_origin_of_time_istringstream{_absolute_origin_of_time_string.c_str()};       // If absolute origin of time too back in time there will be errors in the trajectories because of time resolution in header.stamp.sec of the PointStamped. If in the future (negative times) there will be errors.
        absolute_origin_of_time_istringstream >> parse(format_xd_dateTime, absolute_origin_of_time);

        // In order to extract data from the XML we use the XML Parser "pugixml" version 1.8.
        pugi::xml_document doc;     // Initialize the variable that will contain the whole XML tree or data structure. XML is treated as XML.

        doc.load_string( _XML_string.c_str() );  // Load the XML into the previous variable, generating the tree or data structure that contains all the raw data of the XML (XML treated as XML).
        // .c_str() method is completely necessary for load_string to compile. c_str() returns returns a char * to an array that contains the elements of the string, if not used then the format of the string cannot be recognised for the load_string function.

        // pugiparser sort the information in a tree of nodes. All the nodes but the root of the tree have one parent and zero or several children. The nodes can have attributes.

        if (std::string(doc.child("event").child("uuid").child_value()).size()==0) {
            // Event uuid is completely necessary, if not given return error.
            empty_event_to_return_if_error.message_if_parse_error = "Parse error, event uuid not found.";
            empty_event_to_return_if_error.element_uuid_if_parse_error = "";
            return empty_event_to_return_if_error;
        } else {
            event_to_return.event_id = doc.child("event").child("uuid").child_value();
            event_to_return.planned_duration = std::string(doc.child("event").child("plannedDuration").child_value()).size()>0 ? (int) std::stod(doc.child("event").child("plannedDuration").child_value(), &sz) : -1 ;
            if (std::string(doc.child("event").child("plannedStartTime").child_value()).size()>0) {
                sys_seconds absolute_event_planned_start_time;
                std::istringstream absolute_event_start_time_istringstream{doc.child("event").child("plannedStartTime").child_value()};
                absolute_event_start_time_istringstream >> parse(format_xd_dateTime, absolute_event_planned_start_time);
                auto event_planned_start_time_relative_to_origin_of_time = absolute_event_planned_start_time - absolute_origin_of_time;
                if ((int) std::chrono::seconds(event_planned_start_time_relative_to_origin_of_time).count()>=0) {
                    if (std::string(doc.child("event").child("timezone").child_value()).size() == 0) {
                        empty_event_to_return_if_error.message_if_parse_error = "Parse error, event timezone not defined.";
                        empty_event_to_return_if_error.element_uuid_if_parse_error = event_to_return.event_id;
                        return empty_event_to_return_if_error;
                    }
                    if (_time_zone.size() > 0) {
                        if (std::string(doc.child("event").child("timezone").child_value()) == _time_zone) {
                            event_to_return.planned_start_time = (int) std::chrono::seconds(event_planned_start_time_relative_to_origin_of_time).count();
                        } else {
                            empty_event_to_return_if_error.message_if_parse_error = "Parse error, event not in the same timezone as the other events.";
                            empty_event_to_return_if_error.element_uuid_if_parse_error = event_to_return.event_id;
                            return empty_event_to_return_if_error;
                        }
                    } else {
                        _time_zone = std::string(doc.child("event").child("timezone").child_value());
                        event_to_return.planned_start_time = (int) std::chrono::seconds(event_planned_start_time_relative_to_origin_of_time).count();
                    }
                } else if ((int) std::chrono::seconds(event_planned_start_time_relative_to_origin_of_time).count() == -1) {
                    event_to_return.planned_start_time = -1;
                }
            } else {
                event_to_return.planned_start_time = -1;
            }
        }

        if (std::string(doc.child("event").attribute("parentEventRef").value()).size()>0) {
            if (_stem_event_id.size()==0) {
                _stem_event_id = std::string(doc.child("event").attribute("parentEventRef").value());
            } else if (std::string(doc.child("event").attribute("parentEventRef").value())!=_stem_event_id) {
                empty_event_to_return_if_error.message_if_parse_error = "Parse error, parentEventRef uuid doesn't match with the one previously given.";
                empty_event_to_return_if_error.element_uuid_if_parse_error = event_to_return.event_id;
                return empty_event_to_return_if_error;
            }
        } else {
            empty_event_to_return_if_error.message_if_parse_error = "Parse error, parentEventRef uuid not found.";
            empty_event_to_return_if_error.element_uuid_if_parse_error = event_to_return.event_id;
            return empty_event_to_return_if_error;
        }

    /////////// DEBUG ///////////
#ifdef DEBUG_XML_PARSER
    std::cout << std::endl << "EVENT_TO_RETURN:" << std::endl;
    std::cout << "event_id " << event_to_return.event_id << std::endl;
    std::cout << "planned_duration " << event_to_return.planned_duration << std::endl;
    std::cout << "planned_start_time " << event_to_return.planned_start_time << std::endl << std::endl;
#endif
    /////////// DEBUG ///////////

    } catch (...) {     // catch any exception
        empty_event_to_return_if_error.message_if_parse_error = "Unidentifed parse error, please recheck everything.";
        empty_event_to_return_if_error.element_uuid_if_parse_error = "";
        return empty_event_to_return_if_error;
    }

    return event_to_return;

}   // end event_XML_parser



// Inline function that do the XML parse job for the version of the XML.
inline MissionStruct mission_XML_parser(const std::string& _XML_string, const std::map<std::string,EventStruct>& _events, geographic_msgs::GeoPoint& _origin_coordinates_geo) {

    MissionStruct mission_to_return;
    MissionStruct empty_mission_to_return_if_error;

    try {

        std::string::size_type sz;  // Used for std::stod for the string to number conversion.

        // In order to extract data from the XML we use the XML Parser "pugixml" version 1.8.
        pugi::xml_document doc;     // Initialize the variable that will contain the whole XML tree or data structure. XML is treated as XML.

        doc.load_string( _XML_string.c_str() );  // Load the XML into the previous variable, generating the tree or data structure that contains all the raw data of the XML (XML treated as XML).
        // .c_str() method is completely necessary for load_string to compile. c_str() returns returns a char * to an array that contains the elements of the string, if not used then the format of the string cannot be recognised for the load_string function.

        // pugiparser sort the information in a tree of nodes. All the nodes but the root of the tree have one parent and zero or several children. The nodes can have attributes.

        if (std::string(doc.child("mission").child("uuid").child_value()).size()==0) {
            // Mission uuid is completely necessary, if not given return error.
            empty_mission_to_return_if_error.message_if_parse_error = "Parse error, mission uuid not found.";
            empty_mission_to_return_if_error.element_uuid_if_parse_error = "";
            return empty_mission_to_return_if_error;
        } else if (std::string(doc.child("mission").attribute("eventRef").value()).size()==0) {
            // Event uuid is completely necessary, if not given return error.
            empty_mission_to_return_if_error.message_if_parse_error = "Parse error, eventRef uuid not found.";
            empty_mission_to_return_if_error.element_uuid_if_parse_error = std::string(doc.child("mission").child("uuid").child_value());
            return empty_mission_to_return_if_error;
        } else if (std::string(doc.child("mission").child("role").child_value()).size()==0) {
            // Mission role is completely necessary, if not given return error.
            empty_mission_to_return_if_error.message_if_parse_error = "Parse error, mission role not found.";
            empty_mission_to_return_if_error.element_uuid_if_parse_error = std::string(doc.child("mission").child("uuid").child_value());
            return empty_mission_to_return_if_error;
        } else {
            mission_to_return.mission_status.mission_id = doc.child("mission").child("uuid").child_value();
            mission_to_return.mission_status.status = multidrone_msgs::MissionStatus::STATUS_ENROLLED;
            mission_to_return.mission_role = doc.child("mission").child("role").child_value();

            mission_to_return.ref_event_id = std::string(doc.child("mission").attribute("eventRef").value());
            if (_events.count(mission_to_return.ref_event_id)==0) { // ref_event_id not enrolled yet
                empty_mission_to_return_if_error.message_if_parse_error = "Parse error, eventRef uuid doesn't match with existing events enrolled previously.";
                empty_mission_to_return_if_error.element_uuid_if_parse_error = mission_to_return.mission_status.mission_id;
                return empty_mission_to_return_if_error;
            }

            // Check if at least one SAS in this mission, if not give an error:
            int number_of_shooting_action_sequences = 0;
            for (pugi::xml_node shooting_action_sequence_node : doc.child("mission").child("shootingActionSequences").children("shootingActionSequence")) {
                number_of_shooting_action_sequences++;
            }
            if (number_of_shooting_action_sequences==0) {
                empty_mission_to_return_if_error.message_if_parse_error = "Parse error, not a single Shooting Action Sequence found.";
                empty_mission_to_return_if_error.element_uuid_if_parse_error = mission_to_return.mission_status.mission_id;
                return empty_mission_to_return_if_error;
            }

            int SAS_counter=0;
            for (pugi::xml_node shooting_action_sequence_node : doc.child("mission").child("shootingActionSequences").children("shootingActionSequence")) {

                // Check if at least one SA in this SAS, if not give an error:
                int number_of_shooting_actions = 0;
                for (pugi::xml_node shooting_action_node : shooting_action_sequence_node.child("shootingActions").children("shootingAction")) {
                    number_of_shooting_actions++;
                }
                if (number_of_shooting_actions==0) {
                    empty_mission_to_return_if_error.message_if_parse_error = "Parse error, Shooting Action Sequence without Shooting Actions.";
                    empty_mission_to_return_if_error.element_uuid_if_parse_error = std::string(shooting_action_sequence_node.child("uuid").child_value());
                    return empty_mission_to_return_if_error;
                }

                std::string SAS_role = shooting_action_sequence_node.child("role").child_value();
                std::vector<multidrone_msgs::ShootingAction> shooting_action_list;
                std::vector<std::string> KML_list;

                int duration_accumulated_actual_SA_seq_int = 0;
                for (pugi::xml_node shooting_action_node : shooting_action_sequence_node.child("shootingActions").children("shootingAction")) {
                    std::string fly_through_uuid = "";      // UUID if a fly through is done in this shooting action.

                    multidrone_msgs::ShootingAction new_shooting_action;
                    new_shooting_action.start_event = mission_to_return.ref_event_id;
                    new_shooting_action.mission_id = mission_to_return.mission_status.mission_id;
                    new_shooting_action.action_sequence_id = shooting_action_sequence_node.child("uuid").child_value();
                    new_shooting_action.action_id = shooting_action_node.child("uuid").child_value();

                    new_shooting_action.estimated_start_time = _events.at(mission_to_return.ref_event_id).planned_start_time==-1 ? -1 : _events.at(mission_to_return.ref_event_id).planned_start_time + duration_accumulated_actual_SA_seq_int ;
                    new_shooting_action.delay_since_event = duration_accumulated_actual_SA_seq_int ;

                    // If no duration is given it means that the actual SA is unbounded, will be executed until the next event. In this case the duration is estimated with the duration of the event.
                    new_shooting_action.duration.data.sec = std::string(shooting_action_node.child("duration").child_value()).size()==0 ? _events.at(mission_to_return.ref_event_id).planned_duration - duration_accumulated_actual_SA_seq_int : (int) std::stod(shooting_action_node.child("duration").child_value(), &sz)==0 ? _events.at(mission_to_return.ref_event_id).planned_duration - duration_accumulated_actual_SA_seq_int : (int) std::stod(shooting_action_node.child("duration").child_value(), &sz) ;
                    duration_accumulated_actual_SA_seq_int += new_shooting_action.duration.data.sec;

                    // new_shooting_action.formation_speed = std::string(shooting_action_node.child("formationSpeed").child_value()).size()==0 ? -1 : (int) std::stod(shooting_action_node.child("formationSpeed").child_value(), &sz);
                    new_shooting_action.formation_speed = std::string(shooting_action_node.child("formationSpeed").child_value()).size()==0 ? -1 : (int) ( (float) std::stod(shooting_action_node.child("formationSpeed").child_value(), &sz)*1000.0/3600.0);   // The dashboard gives the speed in km/h !

                    bool enter_in_the_loop = false;
                    for (pugi::xml_node target_type_node : doc.child("mission").child("targetTypes").children("targetType")) {
                        enter_in_the_loop = true;
                        if (std::string(target_type_node.child("name").child_value()) == "TARGET_NONE") {
                            new_shooting_action.target_type.type = multidrone_msgs::TargetType::TARGET_NONE;
                        } else if (std::string(target_type_node.child("name").child_value()) == "TARGET_BOAT") {
                            new_shooting_action.target_type.type = multidrone_msgs::TargetType::TARGET_BOAT;
                        } else if (std::string(target_type_node.child("name").child_value()) == "TARGET_CYCLIST") {
                            new_shooting_action.target_type.type = multidrone_msgs::TargetType::TARGET_CYCLIST;
                        } else if (std::string(target_type_node.child("name").child_value()) == "TARGET_RUNNER") {
                            new_shooting_action.target_type.type = multidrone_msgs::TargetType::TARGET_RUNNER;
                        } else if (std::string(target_type_node.child("name").child_value()) == "TARGET_MONUMENT") {
                            new_shooting_action.target_type.type = multidrone_msgs::TargetType::TARGET_MONUMENT;
                        } else {
                            mission_to_return.message_if_parse_error += " Warning: targetType not given in the XML, assuming TARGET_NONE.";
                            new_shooting_action.target_type.type = multidrone_msgs::TargetType::TARGET_NONE;
                        }
                        break;  // Expecting only one targetType, ignore the rest.
                    }
                    if (!enter_in_the_loop) {
                        mission_to_return.message_if_parse_error += " Warning: targetType not given in the XML, assuming TARGET_NONE.";
                        new_shooting_action.target_type.type = multidrone_msgs::TargetType::TARGET_NONE;
                    }

                    geographic_msgs::GeoPoint origin_of_rt_trajectory_geo;
                    geometry_msgs::Point32 origin_of_rt_trajectory_cartesian_32;
                    bool only_once = true;
                    for (pugi::xml_node rt_trajectory_point_node : shooting_action_node.child("RTTrajectory").children("RTTrajectoryPoint")) {

                        geographic_msgs::GeoPoint rt_trajectory_point_geo;
                        geometry_msgs::PointStamped rt_trajectory_point_cartesian_stamped;
                        if (std::string(rt_trajectory_point_node.child("latitude").child_value()).size()>0 && std::string(rt_trajectory_point_node.child("longitude").child_value()).size()>0 ) {
                            rt_trajectory_point_geo.latitude  = (float) std::stod(rt_trajectory_point_node.child("latitude").child_value(), &sz);
                            rt_trajectory_point_geo.longitude = (float) std::stod(rt_trajectory_point_node.child("longitude").child_value(), &sz);
                            rt_trajectory_point_geo.altitude  = 0;

                            geometry_msgs::Point32 rt_trajectory_point_cartesian_32;
                            if (_origin_coordinates_geo.latitude==0 && _origin_coordinates_geo.longitude==0 && _origin_coordinates_geo.altitude==0) {
                                // No origin of coordinate given (pre-production):
                                _origin_coordinates_geo = rt_trajectory_point_geo;   // The first waypoint found in the first mission enrolled will be considered the geographic origin of coordinate.
                                rt_trajectory_point_cartesian_32.x = 0;                 // Current rt_trajectory_point is the Cartesian origin of coordinate.
                                rt_trajectory_point_cartesian_32.y = 0;
                                rt_trajectory_point_cartesian_32.z = 0;
                            } else {
                                // There is a origin of coordinate, so the points in Cartesian coordinates are relative to that origin:
                                rt_trajectory_point_cartesian_32 = geographic_to_cartesian (rt_trajectory_point_geo, _origin_coordinates_geo);
                            }

                            rt_trajectory_point_cartesian_stamped.point.x = rt_trajectory_point_cartesian_32.x;
                            rt_trajectory_point_cartesian_stamped.point.y = rt_trajectory_point_cartesian_32.y;
                            rt_trajectory_point_cartesian_stamped.point.z = 0;
                        } else if (std::string(rt_trajectory_point_node.child("x").child_value()).size()>0 && std::string(rt_trajectory_point_node.child("y").child_value()).size()>0 ) {
                            rt_trajectory_point_cartesian_stamped.point.x = (float) std::stod(rt_trajectory_point_node.child("x").child_value(), &sz);
                            rt_trajectory_point_cartesian_stamped.point.y = (float) std::stod(rt_trajectory_point_node.child("y").child_value(), &sz);
                            rt_trajectory_point_cartesian_stamped.point.z = 0;
                        }

                        if (only_once) {
                            origin_of_rt_trajectory_geo = rt_trajectory_point_geo;
                            origin_of_rt_trajectory_cartesian_32.x = rt_trajectory_point_cartesian_stamped.point.x;
                            origin_of_rt_trajectory_cartesian_32.y = rt_trajectory_point_cartesian_stamped.point.y;
                            origin_of_rt_trajectory_cartesian_32.z = rt_trajectory_point_cartesian_stamped.point.z;
                            only_once = false;
                        }

                        new_shooting_action.rt_trajectory.push_back(rt_trajectory_point_cartesian_stamped);
                    }
                    // Check if at least one rt_trajectory point has been inserted, if not give an error:
                    if (new_shooting_action.rt_trajectory.size()==0) {
                        empty_mission_to_return_if_error.message_if_parse_error = "Parse error, Shooting Action without at least one RTTrajectory point.";
                        empty_mission_to_return_if_error.element_uuid_if_parse_error = std::string(shooting_action_node.child("uuid").child_value());
                        return empty_mission_to_return_if_error;
                    }

                    if ( std::string(shooting_action_node.child("originOfFormation").child("latitude").child_value()).size()>0 && std::string(shooting_action_node.child("originOfFormation").child("longitude").child_value()).size()>0 ) {
                        geographic_msgs::GeoPoint origin_of_formation_geo;
                        origin_of_formation_geo.latitude  = (float) std::stod(shooting_action_node.child("originOfFormation").child("latitude").child_value(), &sz);
                        origin_of_formation_geo.longitude = (float) std::stod(shooting_action_node.child("originOfFormation").child("longitude").child_value(), &sz);
                        origin_of_formation_geo.altitude  = 0;

                        geometry_msgs::Point32 rt_displacement = geographic_to_cartesian (origin_of_formation_geo, origin_of_rt_trajectory_geo);
                        new_shooting_action.rt_displacement.x = rt_displacement.x;
                        new_shooting_action.rt_displacement.y = rt_displacement.y;
                        new_shooting_action.rt_displacement.z = 0;
                    } else if ( std::string(shooting_action_node.child("originOfFormation").child("x").child_value()).size()>0 && std::string(shooting_action_node.child("originOfFormation").child("y").child_value()).size()>0 ) {
                        new_shooting_action.rt_displacement.x = (float) std::stod(shooting_action_node.child("originOfFormation").child("x").child_value(), &sz) - origin_of_rt_trajectory_cartesian_32.x;
                        new_shooting_action.rt_displacement.y = (float) std::stod(shooting_action_node.child("originOfFormation").child("y").child_value(), &sz) - origin_of_rt_trajectory_cartesian_32.y;
                        new_shooting_action.rt_displacement.z = 0;
                    } else {
                        // Origin of formation not given, give an error:
                        empty_mission_to_return_if_error.message_if_parse_error = "Parse error, Shooting Action without originOfFormation.";
                        empty_mission_to_return_if_error.element_uuid_if_parse_error = std::string(shooting_action_node.child("uuid").child_value());
                        return empty_mission_to_return_if_error;
                    }

                    new_shooting_action.length = 0;
                    for (int i = 0; i < new_shooting_action.rt_trajectory.size() - 1; i++) {
                        new_shooting_action.length += sqrt(pow(new_shooting_action.rt_trajectory.at(i + 1).point.x - new_shooting_action.rt_trajectory.at(i).point.x, 2) + pow(new_shooting_action.rt_trajectory.at(i + 1).point.y - new_shooting_action.rt_trajectory.at(i).point.y, 2) + pow(new_shooting_action.rt_trajectory.at(i + 1).point.z - new_shooting_action.rt_trajectory.at(i).point.z, 2));
                    }

                    new_shooting_action.rt_mode = 0;
                    new_shooting_action.rt_id = 0;

                    // Check if at least one SA Role in this SA, if not give an error:
                    int number_of_shooting_action_roles = 0;
                    for (pugi::xml_node shooting_role_node : shooting_action_node.child("shootingRoles").children("shootingRole")) {
                        number_of_shooting_action_roles++;
                    }
                    if (number_of_shooting_action_roles==0) {
                        empty_mission_to_return_if_error.message_if_parse_error = "Parse error, Shooting Action without Shooting Role.";
                        empty_mission_to_return_if_error.element_uuid_if_parse_error = std::string(shooting_action_node.child("uuid").child_value());
                        return empty_mission_to_return_if_error;
                    }

                    for (pugi::xml_node shooting_role_node : shooting_action_node.child("shootingRoles").children("shootingRole")) {
                        multidrone_msgs::ShootingRole new_shooting_role;

                        if (std::string(shooting_role_node.child("name").child_value()) == "STATIC") {
                            new_shooting_role.shooting_type.type = multidrone_msgs::ShootingType::SHOOT_TYPE_STATIC;
                        } else if (std::string(shooting_role_node.child("name").child_value()) == "FLY_THROUGH") {
                            new_shooting_role.shooting_type.type = multidrone_msgs::ShootingType::SHOOT_TYPE_FLY_THROUGH;
                        } else if (std::string(shooting_role_node.child("name").child_value()) == "ESTABLISH") {
                            new_shooting_role.shooting_type.type = multidrone_msgs::ShootingType::SHOOT_TYPE_ESTABLISH;
                        } else if (std::string(shooting_role_node.child("name").child_value()) == "CHASE") {
                            new_shooting_role.shooting_type.type = multidrone_msgs::ShootingType::SHOOT_TYPE_CHASE;
                        } else if (std::string(shooting_role_node.child("name").child_value()) == "LEAD") {
                            new_shooting_role.shooting_type.type = multidrone_msgs::ShootingType::SHOOT_TYPE_LEAD;
                        } else if (std::string(shooting_role_node.child("name").child_value()) == "FLYBY") {
                            new_shooting_role.shooting_type.type = multidrone_msgs::ShootingType::SHOOT_TYPE_FLYBY;
                        } else if (std::string(shooting_role_node.child("name").child_value()) == "LATERAL") {
                            new_shooting_role.shooting_type.type = multidrone_msgs::ShootingType::SHOOT_TYPE_LATERAL;
                        } else if (std::string(shooting_role_node.child("name").child_value()) == "ELEVATOR") {
                            new_shooting_role.shooting_type.type = multidrone_msgs::ShootingType::SHOOT_TYPE_ELEVATOR;
                        } else if (std::string(shooting_role_node.child("name").child_value()) == "ORBIT") {
                            new_shooting_role.shooting_type.type = multidrone_msgs::ShootingType::SHOOT_TYPE_ORBIT;
                        } else {
                            empty_mission_to_return_if_error.message_if_parse_error = "Parse error, Shooting Role without ShootingType.";
                            empty_mission_to_return_if_error.element_uuid_if_parse_error = std::string(shooting_role_node.child("uuid").child_value());
                            return empty_mission_to_return_if_error;
                        }

                        if (std::string(shooting_role_node.child("framingTypeName").child_value()) == "FRAMING_TYPE_ELS") {          // Extreme Long Shot
                            new_shooting_role.framing_type.type = multidrone_msgs::FramingType::FRAMING_TYPE_ELS;
                        } else if (std::string(shooting_role_node.child("framingTypeName").child_value()) == "FRAMING_TYPE_VLS") {   // Very Long Shot
                            new_shooting_role.framing_type.type = multidrone_msgs::FramingType::FRAMING_TYPE_VLS;
                        } else if (std::string(shooting_role_node.child("framingTypeName").child_value()) == "FRAMING_TYPE_LS") {    // Long Shot
                            new_shooting_role.framing_type.type = multidrone_msgs::FramingType::FRAMING_TYPE_LS;
                        } else if (std::string(shooting_role_node.child("framingTypeName").child_value()) == "FRAMING_TYPE_MS") {    // Medium Shot
                            new_shooting_role.framing_type.type = multidrone_msgs::FramingType::FRAMING_TYPE_MS;
                        } else if (std::string(shooting_role_node.child("framingTypeName").child_value()) == "FRAMING_TYPE_MCU") {   // Medium Close-Up
                            new_shooting_role.framing_type.type = multidrone_msgs::FramingType::FRAMING_TYPE_MCU;
                        } else if (std::string(shooting_role_node.child("framingTypeName").child_value()) == "FRAMING_TYPE_CU") {    // Close-Up
                            new_shooting_role.framing_type.type = multidrone_msgs::FramingType::FRAMING_TYPE_CU;
                        } else if (std::string(shooting_role_node.child("framingTypeName").child_value()) == "FRAMING_TYPE_OTS") {   // Over-the-Shoulder
                            new_shooting_role.framing_type.type = multidrone_msgs::FramingType::FRAMING_TYPE_OTS;
                        } else {
                            empty_mission_to_return_if_error.message_if_parse_error = "Parse error, Shooting Role without framingTypeName.";
                            empty_mission_to_return_if_error.element_uuid_if_parse_error = std::string(shooting_role_node.child("uuid").child_value());
                            return empty_mission_to_return_if_error;
                        }

                        if (std::string(shooting_role_node.child("targetIdentifier").child("name").child_value()) == "NONE") {
                            new_shooting_role.target_identifier_type.type = multidrone_msgs::TargetIdentifierType::NONE;
                        } else if (std::string(shooting_role_node.child("targetIdentifier").child("name").child_value()) == "VIRTUAL") {
                            new_shooting_role.target_identifier_type.type = multidrone_msgs::TargetIdentifierType::VIRTUAL;
                        } else if (std::string(shooting_role_node.child("targetIdentifier").child("name").child_value()) == "GPS") {
                            new_shooting_role.target_identifier_type.type = multidrone_msgs::TargetIdentifierType::GPS;
                        } else if (std::string(shooting_role_node.child("targetIdentifier").child("name").child_value()) == "VISUAL") {
                            new_shooting_role.target_identifier_type.type = multidrone_msgs::TargetIdentifierType::VISUAL;
                        } else if (std::string(shooting_role_node.child("targetIdentifier").child("name").child_value()) == "STATIC") {
                            new_shooting_role.target_identifier_type.type = multidrone_msgs::TargetIdentifierType::STATIC;
                        } else if (std::string(shooting_role_node.child("targetIdentifier").child("name").child_value()) == "VISUAL_GPS") {
                            new_shooting_role.target_identifier_type.type = multidrone_msgs::TargetIdentifierType::VISUAL_GPS;
                        } else {
                            empty_mission_to_return_if_error.message_if_parse_error = "Parse error, Shooting Role without targetIdentifier.name.";
                            empty_mission_to_return_if_error.element_uuid_if_parse_error = std::string(shooting_role_node.child("uuid").child_value());
                            return empty_mission_to_return_if_error;
                        }

                        try { // try to parse this
                            json json_shooting_parameters = json::parse( std::string(shooting_role_node.child("params").child_value()) );

                            try { // try to parse this
                                multidrone_msgs::ShootingParameter new_shooting_parameter_z_0;
                                new_shooting_parameter_z_0.param = "z_0";
                                new_shooting_parameter_z_0.value = json_shooting_parameters.at("z_0").at("value").get<float>();
                                new_shooting_role.shooting_parameters.push_back(new_shooting_parameter_z_0);
                            } catch (json::exception &e) { }

                            try { // try to parse this
                                multidrone_msgs::ShootingParameter new_shooting_parameter_x_s;
                                new_shooting_parameter_x_s.param = "x_s";
                                new_shooting_parameter_x_s.value = json_shooting_parameters.at("x_s").at("value").get<float>();
                                new_shooting_role.shooting_parameters.push_back(new_shooting_parameter_x_s);
                            } catch (json::exception &e) { }

                            try { // try to parse this
                                multidrone_msgs::ShootingParameter new_shooting_parameter_x_e;
                                new_shooting_parameter_x_e.param = "x_e";
                                new_shooting_parameter_x_e.value = json_shooting_parameters.at("x_e").at("value").get<float>();
                                new_shooting_role.shooting_parameters.push_back(new_shooting_parameter_x_e);
                            } catch (json::exception &e) { }

                            try { // try to parse this
                                multidrone_msgs::ShootingParameter new_shooting_parameter_z_s;
                                new_shooting_parameter_z_s.param = "z_s";
                                new_shooting_parameter_z_s.value = json_shooting_parameters.at("z_s").at("value").get<float>();
                                new_shooting_role.shooting_parameters.push_back(new_shooting_parameter_z_s);
                            } catch (json::exception &e) { }

                            try { // try to parse this
                                multidrone_msgs::ShootingParameter new_shooting_parameter_z_e;
                                new_shooting_parameter_z_e.param = "z_e";
                                new_shooting_parameter_z_e.value = json_shooting_parameters.at("z_e").at("value").get<float>();
                                new_shooting_role.shooting_parameters.push_back(new_shooting_parameter_z_e);
                            } catch (json::exception &e) { }

                            try { // try to parse this
                                multidrone_msgs::ShootingParameter new_shooting_parameter_y_0;
                                new_shooting_parameter_y_0.param = "y_0";
                                new_shooting_parameter_y_0.value = json_shooting_parameters.at("y_0").at("value").get<float>();
                                new_shooting_role.shooting_parameters.push_back(new_shooting_parameter_y_0);
                            } catch (json::exception &e) { }

                            try { // try to parse this
                                multidrone_msgs::ShootingParameter new_shooting_parameter_r_0;
                                new_shooting_parameter_r_0.param = "r_0";
                                new_shooting_parameter_r_0.value = json_shooting_parameters.at("r_0").at("value").get<float>();
                                new_shooting_role.shooting_parameters.push_back(new_shooting_parameter_r_0);
                            } catch (json::exception &e) { }

                            try { // try to parse this
                                multidrone_msgs::ShootingParameter new_shooting_parameter_azimuth_s;
                                new_shooting_parameter_azimuth_s.param = "azimuth_s";
                                new_shooting_parameter_azimuth_s.value = json_shooting_parameters.at("azimuth_s").at("value").get<float>();
                                new_shooting_role.shooting_parameters.push_back(new_shooting_parameter_azimuth_s);
                            } catch (json::exception &e) { }

                            try { // try to parse this
                                multidrone_msgs::ShootingParameter new_shooting_parameter_azimuth_speed;
                                new_shooting_parameter_azimuth_speed.param = "azimuth_speed";
                                new_shooting_parameter_azimuth_speed.value = json_shooting_parameters.at("azimuth_speed").at("value").get<float>();
                                new_shooting_role.shooting_parameters.push_back(new_shooting_parameter_azimuth_speed);
                            } catch (json::exception &e) { }

                        } catch (json::exception &e) {}

                        try { // try to parse this
                            json json_targetIdentifierParams = json::parse( std::string(shooting_role_node.child("targetIdentifierParams").child_value()) );

                            unsigned int new_rt_mode = 0;
                            unsigned int current_id = 0;

                            try { // try to parse this
                                multidrone_msgs::ShootingParameter new_target_parameter_ID;
                                new_target_parameter_ID.param = "ID";
                                new_target_parameter_ID.value = json_targetIdentifierParams.at("ID").get<float>();
                                new_shooting_role.target_parameters.push_back(new_target_parameter_ID);
                                current_id = (unsigned int) new_target_parameter_ID.value;
                            } catch (json::exception &e) {}

                            try { // try to parse this
                                new_rt_mode = json_targetIdentifierParams.at("rt_mode").get<unsigned int>();
                                if (new_rt_mode > 0) {
                                    if (new_shooting_action.rt_mode > 0) {  // If there is already a rt_mode different that default
                                        mission_to_return.message_if_parse_error += " Warning: more than one Shooting Role with rt_mode>0 in the same Shooting Action, assigning the last one to rt_mode and rt_id.";
                                    }
                                    new_shooting_action.rt_mode = new_rt_mode;
                                    new_shooting_action.rt_id = current_id;
                                }
                            } catch (json::exception &e) {}

                            try { // try to parse this
                                multidrone_msgs::ShootingParameter new_target_parameter_Latitude;
                                new_target_parameter_Latitude.param = "Latitude";
                                new_target_parameter_Latitude.value = json_targetIdentifierParams.at("Latitude").get<float>();
                                new_shooting_role.target_parameters.push_back(new_target_parameter_Latitude);
                            } catch (json::exception &e) {}

                            try { // try to parse this
                                multidrone_msgs::ShootingParameter new_target_parameter_Longitude;
                                new_target_parameter_Longitude.param = "Longitude";
                                new_target_parameter_Longitude.value = json_targetIdentifierParams.at("Longitude").get<float>();
                                new_shooting_role.target_parameters.push_back(new_target_parameter_Longitude);
                            } catch (json::exception &e) {}

                            try { // try to parse this
                                multidrone_msgs::ShootingParameter new_target_parameter_Altitude;
                                new_target_parameter_Altitude.param = "Altitude";
                                new_target_parameter_Altitude.value = json_targetIdentifierParams.at("Altitude").get<float>();
                                new_shooting_role.target_parameters.push_back(new_target_parameter_Altitude);
                            } catch (json::exception &e) {}

                            try { // try to parse this
                                multidrone_msgs::ShootingParameter new_target_parameter_pan_s;
                                new_target_parameter_pan_s.param = "pan_s";
                                new_target_parameter_pan_s.value = json_targetIdentifierParams.at("pan_s").get<float>();
                                new_shooting_role.target_parameters.push_back(new_target_parameter_pan_s);
                            } catch (json::exception &e) {}

                            try { // try to parse this
                                multidrone_msgs::ShootingParameter new_target_parameter_tilt_s;
                                new_target_parameter_tilt_s.param = "tilt_s";
                                new_target_parameter_tilt_s.value = json_targetIdentifierParams.at("tilt_s").get<float>();
                                new_shooting_role.target_parameters.push_back(new_target_parameter_tilt_s);
                            } catch (json::exception &e) {}

                            try { // try to parse this
                                multidrone_msgs::ShootingParameter new_target_parameter_pan_e;
                                new_target_parameter_pan_e.param = "pan_e";
                                new_target_parameter_pan_e.value = json_targetIdentifierParams.at("pan_e").get<float>();
                                new_shooting_role.target_parameters.push_back(new_target_parameter_pan_e);
                            } catch (json::exception &e) {}

                            try { // try to parse this
                                multidrone_msgs::ShootingParameter new_target_parameter_tilt_e;
                                new_target_parameter_tilt_e.param = "tilt_e";
                                new_target_parameter_tilt_e.value = json_targetIdentifierParams.at("tilt_e").get<float>();
                                new_shooting_role.target_parameters.push_back(new_target_parameter_tilt_e);
                            } catch (json::exception &e) {}

                        } catch (json::exception &e) {}


                        // Check that all the necessary shooting_parameters for this shooting_type are given:
                        std::map<std::string,float> shooting_parameters_map;
                        for(int i = 0; i<new_shooting_role.shooting_parameters.size(); i++) {
                            shooting_parameters_map[new_shooting_role.shooting_parameters.at(i).param] = new_shooting_role.shooting_parameters.at(i).value;
                        }
                        if (new_shooting_role.shooting_type.type == multidrone_msgs::ShootingType::SHOOT_TYPE_STATIC) {
                            if (shooting_parameters_map.count("z_0")==0) {
                                empty_mission_to_return_if_error.message_if_parse_error = "Parse error, not enough shooting_parameters in a Shooting Role with ShootingType STATIC.";
                                empty_mission_to_return_if_error.element_uuid_if_parse_error = std::string(shooting_role_node.child("uuid").child_value());
                                return empty_mission_to_return_if_error;
                            }
                        } else if (new_shooting_role.shooting_type.type == multidrone_msgs::ShootingType::SHOOT_TYPE_FLY_THROUGH) {
                            fly_through_uuid = std::string(shooting_role_node.child("uuid").child_value());
                            if (shooting_parameters_map.count("z_0")==0) {
                                empty_mission_to_return_if_error.message_if_parse_error = "Parse error, not enough shooting_parameters in a Shooting Role with ShootingType FLY_THROUGH.";
                                empty_mission_to_return_if_error.element_uuid_if_parse_error = std::string(shooting_role_node.child("uuid").child_value());
                                return empty_mission_to_return_if_error;
                            }
                        } else if (new_shooting_role.shooting_type.type == multidrone_msgs::ShootingType::SHOOT_TYPE_ESTABLISH) {
                            if (shooting_parameters_map.count("x_s")==0 || shooting_parameters_map.count("x_e")==0 || shooting_parameters_map.count("z_s")==0 || shooting_parameters_map.count("z_e")==0) {
                                empty_mission_to_return_if_error.message_if_parse_error = "Parse error, not enough shooting_parameters in a Shooting Role with ShootingType ESTABLISH.";
                                empty_mission_to_return_if_error.element_uuid_if_parse_error = std::string(shooting_role_node.child("uuid").child_value());
                                return empty_mission_to_return_if_error;
                            }
                        } else if (new_shooting_role.shooting_type.type == multidrone_msgs::ShootingType::SHOOT_TYPE_CHASE) {
                            if (shooting_parameters_map.count("x_s")==0 || shooting_parameters_map.count("x_e")==0 || shooting_parameters_map.count("z_0")==0) {
                                empty_mission_to_return_if_error.message_if_parse_error = "Parse error, not enough shooting_parameters in a Shooting Role with ShootingType CHASE.";
                                empty_mission_to_return_if_error.element_uuid_if_parse_error = std::string(shooting_role_node.child("uuid").child_value());
                                return empty_mission_to_return_if_error;
                            }
                        } else if (new_shooting_role.shooting_type.type == multidrone_msgs::ShootingType::SHOOT_TYPE_LEAD) {
                            if (shooting_parameters_map.count("x_s")==0 || shooting_parameters_map.count("x_e")==0 || shooting_parameters_map.count("z_0")==0) {
                                empty_mission_to_return_if_error.message_if_parse_error = "Parse error, not enough shooting_parameters in a Shooting Role with ShootingType LEAD.";
                                empty_mission_to_return_if_error.element_uuid_if_parse_error = std::string(shooting_role_node.child("uuid").child_value());
                                return empty_mission_to_return_if_error;
                            }
                        } else if (new_shooting_role.shooting_type.type == multidrone_msgs::ShootingType::SHOOT_TYPE_FLYBY) {
                            if (shooting_parameters_map.count("x_s")==0 || shooting_parameters_map.count("x_e")==0 || shooting_parameters_map.count("y_0")==0 || shooting_parameters_map.count("z_0")==0) {
                                empty_mission_to_return_if_error.message_if_parse_error = "Parse error, not enough shooting_parameters in a Shooting Role with ShootingType FLYBY.";
                                empty_mission_to_return_if_error.element_uuid_if_parse_error = std::string(shooting_role_node.child("uuid").child_value());
                                return empty_mission_to_return_if_error;
                            }
                        } else if (new_shooting_role.shooting_type.type == multidrone_msgs::ShootingType::SHOOT_TYPE_LATERAL) {
                            if (shooting_parameters_map.count("y_0")==0 || shooting_parameters_map.count("z_0")==0) {
                                empty_mission_to_return_if_error.message_if_parse_error = "Parse error, not enough shooting_parameters in a Shooting Role with ShootingType LATERAL.";
                                empty_mission_to_return_if_error.element_uuid_if_parse_error = std::string(shooting_role_node.child("uuid").child_value());
                                return empty_mission_to_return_if_error;
                            }
                        } else if (new_shooting_role.shooting_type.type == multidrone_msgs::ShootingType::SHOOT_TYPE_ELEVATOR) {
                            if (shooting_parameters_map.count("z_s")==0 || shooting_parameters_map.count("z_e")==0) {
                                empty_mission_to_return_if_error.message_if_parse_error = "Parse error, not enough shooting_parameters in a Shooting Role with ShootingType ELEVATOR.";
                                empty_mission_to_return_if_error.element_uuid_if_parse_error = std::string(shooting_role_node.child("uuid").child_value());
                                return empty_mission_to_return_if_error;
                            }
                        } else if (new_shooting_role.shooting_type.type == multidrone_msgs::ShootingType::SHOOT_TYPE_ORBIT) {
                            if (shooting_parameters_map.count("r_0")==0 || shooting_parameters_map.count("azimuth_s")==0 || shooting_parameters_map.count("z_0")==0 || shooting_parameters_map.count("azimuth_speed")==0) {
                                empty_mission_to_return_if_error.message_if_parse_error = "Parse error, not enough shooting_parameters in a Shooting Role with ShootingType ORBIT.";
                                empty_mission_to_return_if_error.element_uuid_if_parse_error = std::string(shooting_role_node.child("uuid").child_value());
                                return empty_mission_to_return_if_error;
                            }
                        }


                        // Check that all the necessary target_parameters for this target_identifier_type are given:
                        std::map<std::string,float> target_parameters_map;
                        for(int i = 0; i<new_shooting_role.target_parameters.size(); i++) {
                            target_parameters_map[new_shooting_role.target_parameters.at(i).param] = new_shooting_role.target_parameters.at(i).value;
                        }
                        if (new_shooting_role.target_identifier_type.type == multidrone_msgs::TargetIdentifierType::GPS) {
                            if (target_parameters_map.count("ID")==0) {
                                empty_mission_to_return_if_error.message_if_parse_error = "Parse error, not enough target_parameters in a Shooting Role with TargetIdentifierType GPS.";
                                empty_mission_to_return_if_error.element_uuid_if_parse_error = std::string(shooting_role_node.child("uuid").child_value());
                                return empty_mission_to_return_if_error;
                            }
                        } else if (new_shooting_role.target_identifier_type.type == multidrone_msgs::TargetIdentifierType::VISUAL_GPS) {
                            if (target_parameters_map.count("ID")==0) {
                                empty_mission_to_return_if_error.message_if_parse_error = "Parse error, not enough target_parameters in a Shooting Role with TargetIdentifierType VISUAL_GPS.";
                                empty_mission_to_return_if_error.element_uuid_if_parse_error = std::string(shooting_role_node.child("uuid").child_value());
                                return empty_mission_to_return_if_error;
                            }
                        } else if (new_shooting_role.target_identifier_type.type == multidrone_msgs::TargetIdentifierType::VISUAL) {
                        } else if (new_shooting_role.target_identifier_type.type == multidrone_msgs::TargetIdentifierType::VIRTUAL) {
                        } else if (new_shooting_role.target_identifier_type.type == multidrone_msgs::TargetIdentifierType::NONE) {
                            if (target_parameters_map.count("pan_s")==0 || target_parameters_map.count("tilt_s")==0 || target_parameters_map.count("pan_e")==0 || target_parameters_map.count("tilt_e")==0) {
                                empty_mission_to_return_if_error.message_if_parse_error = "Parse error, not enough target_parameters in a Shooting Role with TargetIdentifierType NONE.";
                                empty_mission_to_return_if_error.element_uuid_if_parse_error = std::string(shooting_role_node.child("uuid").child_value());
                                return empty_mission_to_return_if_error;
                            }
                        } else if (new_shooting_role.target_identifier_type.type == multidrone_msgs::TargetIdentifierType::STATIC) {
                            if (target_parameters_map.count("Latitude")==0 || target_parameters_map.count("Longitude")==0 || target_parameters_map.count("Altitude")==0) {
                                empty_mission_to_return_if_error.message_if_parse_error = "Parse error, not enough target_parameters in a Shooting Role with TargetIdentifierType STATIC.";
                                empty_mission_to_return_if_error.element_uuid_if_parse_error = std::string(shooting_role_node.child("uuid").child_value());
                                return empty_mission_to_return_if_error;
                            } else {
                                // The position of the TargetIdentifierType STATIC will be needed in Cartesian coordinates, so two new parameters (x,y) are inserted into target_parameters:
                                geographic_msgs::GeoPoint static_point_geo;
                                static_point_geo.latitude  = target_parameters_map["Latitude"];
                                static_point_geo.longitude = target_parameters_map["Longitude"];
                                static_point_geo.altitude  = 0;

                                geometry_msgs::Point32 static_point_cartesian = geographic_to_cartesian (static_point_geo, _origin_coordinates_geo);

                                multidrone_msgs::ShootingParameter new_target_parameter_x;
                                new_target_parameter_x.param = "x";
                                new_target_parameter_x.value = static_point_cartesian.x;
                                new_shooting_role.target_parameters.push_back(new_target_parameter_x);

                                multidrone_msgs::ShootingParameter new_target_parameter_y;
                                new_target_parameter_y.param = "y";
                                new_target_parameter_y.value = static_point_cartesian.y;
                                new_shooting_role.target_parameters.push_back(new_target_parameter_y);
                            }
                        }

                        new_shooting_action.shooting_roles.push_back(new_shooting_role);
                    }

                    new_shooting_action.ndrones = new_shooting_action.shooting_roles.size();

                    std::string KML_of_this_SA ="";
                    for (pugi::xml_node map_node : shooting_action_node.child("Maps").children("Map")) {
                        json json_map = json::parse( std::string(map_node.child("description").child_value()) );

                        try { // try to parse this
                            KML_of_this_SA = json_map.at("KML").get<std::string>();
                        } catch (json::exception &e) { }

                        break;  // Expecting only one Map, ignore the rest.
                    }
                    KML_list.push_back(KML_of_this_SA);
                    if (KML_of_this_SA.size()<3) {
                        mission_to_return.message_if_parse_error += " Warning: KML empty in at least one SA.";
                    }


                    // Check that formation speed is given if rt_mode is 0:
                    if (new_shooting_action.rt_mode==0 && new_shooting_action.formation_speed==-1) {
                        empty_mission_to_return_if_error.message_if_parse_error = "Parse error, rt_mode is 0 and no formation speed provided.";
                        empty_mission_to_return_if_error.element_uuid_if_parse_error = std::string(shooting_action_node.child("uuid").child_value());
                        return empty_mission_to_return_if_error;
                    }

                    // Check if rt_mode > 0 with a fly through:
                    if (fly_through_uuid != "" && new_shooting_action.rt_mode > 0) {
                        empty_mission_to_return_if_error.message_if_parse_error = "Parse error, rt_mode > 0 in a FLY_THROUGH.";
                        empty_mission_to_return_if_error.element_uuid_if_parse_error = fly_through_uuid;
                        return empty_mission_to_return_if_error;
                    }

                    shooting_action_list.push_back( new_shooting_action );
                }

                if (mission_to_return.sequence_role_and_shooting_actions.count(SAS_role)>0)
                    mission_to_return.sequence_role_and_shooting_actions.at(SAS_role).insert( mission_to_return.sequence_role_and_shooting_actions.at(SAS_role).end(), shooting_action_list.begin(), shooting_action_list.end() );
                else
                    mission_to_return.sequence_role_and_shooting_actions[SAS_role] = shooting_action_list;

                if (mission_to_return.KML_strings_for_each_SA.count(SAS_role)>0)
                    mission_to_return.KML_strings_for_each_SA.at(SAS_role).insert( mission_to_return.KML_strings_for_each_SA.at(SAS_role).end(), KML_list.begin(), KML_list.end() );
                else
                    mission_to_return.KML_strings_for_each_SA[SAS_role] = KML_list;

                if (mission_to_return.SASs_id.count(SAS_role)>0)
                    mission_to_return.SASs_id.at(SAS_role).push_back(shooting_action_list.at(0).action_sequence_id);
                else
                    mission_to_return.SASs_id[SAS_role].push_back(shooting_action_list.at(0).action_sequence_id);

                SAS_counter++;
            }
        }

    /////////// DEBUG ///////////
#ifdef DEBUG_XML_PARSER
    std::cout << std::endl << "MISSION_TO_RETURN:" << std::endl;
    std::cout << "ref_event_id " << mission_to_return.ref_event_id << std::endl;
    std::cout << "mission_role " << mission_to_return.mission_role << std::endl;
    std::cout << "mission_status:" << std::endl;
    std::cout << "  mission_id " << mission_to_return.mission_status.mission_id << std::endl;
    if ( mission_to_return.mission_status.status == multidrone_msgs::MissionStatus::STATUS_ENROLLED )         std::cout << "  status STATUS_ENROLLED" << std::endl;
    else if ( mission_to_return.mission_status.status == multidrone_msgs::MissionStatus::STATUS_PLANNING )    std::cout << "  status STATUS_PLANNING" << std::endl;
    else if ( mission_to_return.mission_status.status == multidrone_msgs::MissionStatus::STATUS_SUPERVISING ) std::cout << "  status STATUS_SUPERVISING" << std::endl;
    else if ( mission_to_return.mission_status.status == multidrone_msgs::MissionStatus::STATUS_REJECTED )    std::cout << "  status STATUS_REJECTED" << std::endl;
    else if ( mission_to_return.mission_status.status == multidrone_msgs::MissionStatus::STATUS_VALIDATED )   std::cout << "  status STATUS_VALIDATED" << std::endl;
    else if ( mission_to_return.mission_status.status == multidrone_msgs::MissionStatus::STATUS_READY )       std::cout << "  status STATUS_READY" << std::endl;
    else if ( mission_to_return.mission_status.status == multidrone_msgs::MissionStatus::STATUS_RUNNING )     std::cout << "  status STATUS_RUNNING" << std::endl;
    else if ( mission_to_return.mission_status.status == multidrone_msgs::MissionStatus::STATUS_ABORTED )     std::cout << "  status STATUS_ABORTED" << std::endl;
    else if ( mission_to_return.mission_status.status == multidrone_msgs::MissionStatus::STATUS_ENDED )       std::cout << "  status STATUS_ENDED" << std::endl;
    for (int i=0; i<mission_to_return.mission_status.error_list.size(); i++) {
        std::cout << "  error_list[" << i << "]:" << std::endl;
        std::cout << "    message " << mission_to_return.mission_status.error_list.at(i).message << std::endl;
        if ( mission_to_return.mission_status.error_list.at(i).type == multidrone_msgs::MissionError::ERROR_TYPE_SAFETY )        std::cout << "    type ERROR_TYPE_SAFETY" << std::endl;
        else if ( mission_to_return.mission_status.error_list.at(i).type == multidrone_msgs::MissionError::ERROR_TYPE_PLANNER )  std::cout << "    type ERROR_TYPE_PLANNER" << std::endl;
    }
    for (std::map< std::string, std::vector<multidrone_msgs::ShootingAction> >::iterator it = mission_to_return.sequence_role_and_shooting_actions.begin(); it != mission_to_return.sequence_role_and_shooting_actions.end(); it++) {
        std::cout << "  sequence_role_and_shooting_actions:" << std::endl;
        std::cout << "    sequence_role " << it->first << std::endl;
        for (int j=0; j<it->second.size(); j++) {
            std::cout << "    shooting_action[" << j << "]:" << std::endl;
            std::cout << it->second.at(j) << std::endl;
        }
    }
    for (std::map< std::string, std::vector<std::string> >::iterator it = mission_to_return.KML_strings_for_each_SA.begin(); it != mission_to_return.KML_strings_for_each_SA.end(); it++) {
        std::cout << "  KML_strings_for_each_SA:" << std::endl;
        std::cout << "    sequence_role " << it->first << std::endl;
        for (int j=0; j<it->second.size(); j++) {
            std::cout << "    KML[" << j << "]:" << std::endl;
            std::cout << it->second.at(j) << std::endl;
        }
    }
    for (std::map< std::string, std::vector<std::string> >::iterator it = mission_to_return.SASs_id.begin(); it != mission_to_return.SASs_id.end(); it++) {
        std::cout << "  SASs_id:" << std::endl;
        std::cout << "    sequence_role " << it->first << std::endl;
        for (int j=0; j<it->second.size(); j++) {
            std::cout << "    SAS_id[" << j << "]:" << std::endl;
            std::cout << it->second.at(j) << std::endl;
        }
    }
#endif
    /////////// DEBUG ///////////

    } catch (...) {     // catch any exception
        empty_mission_to_return_if_error.message_if_parse_error = "Unidentifed parse error, please recheck everything.";
        empty_mission_to_return_if_error.element_uuid_if_parse_error = "";
        return empty_mission_to_return_if_error;
    }

    return mission_to_return;

}   // end mission_XML_parser


}}  // end namespace multidrone::XMLparser

#endif  // XML_PARSER_H