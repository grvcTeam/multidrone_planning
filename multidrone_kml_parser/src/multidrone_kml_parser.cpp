/**
 * MULTIDRONE Project:
 *
 * Multidrone KML parser.
 *
 * Made using the XML Parser "pugixml", version 1.8, which has the MIT License and the following Copyright:
 *
 * Copyright (c) 2006-2017 Arseny Kapoulkine
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
 * This software is based on pugixml library (http://pugixml.org).
 * pugixml is Copyright (C) 2006-2017 Arseny Kapoulkine.
 *
 */

#include <multidrone_kml_parser/multidrone_kml_parser.h>

#include <iostream>
#include <limits>
#include <algorithm>

#include "multidrone_kml_parser/pugixml.hpp"
#include "multidrone_kml_parser/utm_to_cartesian.hpp"
#include "multidrone_kml_parser/geographic_to_cartesian.hpp"

namespace multidrone {

// Brief Constructor: default constructor, does nothing.
KMLparser::KMLparser() {}



// Brief Constructor: overloaded constructor that receives the KML sting and call the parser method.
KMLparser::KMLparser(const std::string& _KML_string, const geographic_msgs::GeoPoint& _origin_coordinates_geo) {
    runParser(_KML_string, _origin_coordinates_geo);
}



// Brief Destructor
KMLparser::~KMLparser() {}



// Method that do the parse job.
void KMLparser::runParser(const std::string& _KML_string, const geographic_msgs::GeoPoint& _origin_coordinates_geo) {

    clearKMLparserClass();

    try { // try to parse this

        // In order to extract data from the KML we use the XML Parser "pugixml" version 1.8.
        pugi::xml_document doc;     // Initialize the variable that will contain the whole XML tree or data structure. KML is treated as XML.

        doc.load_string( _KML_string.c_str() );  // Load the XML into the previous variable, generating the tree or data structure that contains all the raw data of the XML (KML treated as XML).
        // .c_str() method is completely necessary for load_string to compile. c_str() returns returns a char * to an array that contains the elements of the string, if not used then the format of the string cannot be recognised for the load_string function.

        // std::cout << std::endl << "Parsing KML with pugixml:" << std::endl;

        // pugiparser sort the information in a tree of nodes. All the nodes but the root of the tree have one parent and zero or several children. The nodes can have attributes, but this is not the case.
        for (pugi::xml_node Folder : doc.child("kml").child("Document").children()) {
            for (pugi::xml_node Placemark : Folder.children()) {

                if ( std::string(Placemark.child("name").child_value()) == "1" ) {
                    // std::cout << "1 Landing Site/Regular Takeoff Site (re-charging/ relay stations)" << std::endl;

                    // string that contains the raw data of coordinates of the point:
                    std::string station_string = std::string( Placemark.child("Point").child("coordinates").child_value() );

                    // Remove new lines if exist in the string:
                    station_string.erase(std::remove(station_string.begin(), station_string.end(), '\n'), station_string.end());
                    station_string.erase(std::remove(station_string.begin(), station_string.end(), '\r'), station_string.end());

                    std::string::size_type sz;
                    geographic_msgs::GeoPoint station_point;

                    // First, erase the blank spaces after and before the coordinates, converting commas ',' into spaces ' ':
                    for ( int i = station_string.size()-1; i>=0; i-- ) {
                        if ( station_string[i]==' ' || station_string[i]=='\t' ) station_string.erase(station_string.begin()+i);
                        else if ( station_string[i]==',' ) station_string[i]=' ';
                    }

                    // With the previous string the data is extracted to double (point is in float64) with stod. The string is divided in the first space ' ' with subst:
                    station_point.longitude = std::stod (station_string,&sz);
                    station_string = station_string.substr(sz);
                    station_point.latitude = std::stod (station_string,&sz);
                    station_point.altitude = std::stod (station_string.substr(sz));

                    stations_geo_.push_back(station_point);


                } else if ( std::string(Placemark.child("name").child_value()) == "2" ) {
                    // std::cout << "2 no-fly zone" << std::endl;

                    // string that contains the raw data of the edges (coordinates) of the polygon:
                    std::string no_fly_zone_string = std::string( Placemark.child("Polygon").child("outerBoundaryIs").child("LinearRing").child("coordinates").child_value() );

                    // Remove new lines if exist in the string:
                    no_fly_zone_string.erase(std::remove(no_fly_zone_string.begin(), no_fly_zone_string.end(), '\n'), no_fly_zone_string.end());
                    no_fly_zone_string.erase(std::remove(no_fly_zone_string.begin(), no_fly_zone_string.end(), '\r'), no_fly_zone_string.end());

                    std::string::size_type sz;
                    geographic_msgs::GeoPoint no_fly_zone_point;
                    std::vector< geographic_msgs::GeoPoint > actual_no_fly_zone;

                    // First, erase the blank space before the coordinates (if exist):
                    int i = 0;
                    while ( no_fly_zone_string[i]==' ' || no_fly_zone_string[i]=='\t' ) {
                        i++;
                        if (i==no_fly_zone_string.size()) break;
                    }
                    i--;
                    for ( int j=0; j<=i; j++ ) no_fly_zone_string.erase(no_fly_zone_string.begin());

                    // Second, erase the blank space after the coordinates (if exist):
                    i = 0;
                    while ( no_fly_zone_string[no_fly_zone_string.size()-1-i]==' ' || no_fly_zone_string[no_fly_zone_string.size()-1-i]=='\t' ) {
                        i++;
                        if (i==-1) break;
                    }
                    i--;
                    for ( int j=0; j<=i; j++ ) no_fly_zone_string.erase(no_fly_zone_string.end()-1);

                    // Third, erase the blank space between pairs of coordinates and insert there a comma ',':
                    i = no_fly_zone_string.size()-1;
                    while ( i>0 ) {
                        if ( (no_fly_zone_string[i-1]==' ' || no_fly_zone_string[i-1]=='\t') && (no_fly_zone_string[i]!=' ' && no_fly_zone_string[i]!='\t') ) {
                            i--;
                            int j=i;
                            while ( j>0 && (no_fly_zone_string[j-1]==' ' || no_fly_zone_string[j-1]=='\t') ) j--; //std::cout << i << j << std::endl;}
                            for ( int k=i-j+1; k>0; k-- ) no_fly_zone_string.erase(no_fly_zone_string.begin()+j);
                            no_fly_zone_string.insert(no_fly_zone_string.begin()+j,',');
                            i=j-1;
                        } else {
                            i--;
                        }
                    }

                    // Fourth, reemplace commas ',' with spaces ' ':
                    for ( int j = 0; j<no_fly_zone_string.size(); j++ ) {
                        if ( no_fly_zone_string[j]==',' ) no_fly_zone_string[j]=' ';
                    }

                    // // DEBUG, show the string with the clean data, ready for the double conversion
                    // std::cout << no_fly_zone_string << std::endl;

                    // Convert the string data, clean and ready, to double (float64 because of point) with stod.
                    // Each time a number is converted (numbers separated by space ' '), the string is overwriten with the rest of the string using substr.
                    // The quantity of numbers have to be multiple of 3.
                    while ( no_fly_zone_string.size()>0 ) {
                        no_fly_zone_point.longitude = std::stod (no_fly_zone_string,&sz);
                        no_fly_zone_string = no_fly_zone_string.substr(sz);
                        no_fly_zone_point.latitude = std::stod (no_fly_zone_string,&sz);
                        no_fly_zone_string = no_fly_zone_string.substr(sz);
                        no_fly_zone_point.altitude = std::stod (no_fly_zone_string,&sz);
                        no_fly_zone_string = no_fly_zone_string.substr(sz);
                        actual_no_fly_zone.push_back(no_fly_zone_point);
                    }

                    // // DEBUG, show the whole actual_no_fly_zone vector of points
                    // for (int j=0; j<actual_no_fly_zone.size(); j++) std::cout << std::setprecision (10) <<actual_no_fly_zone[j];
                    // std::cout << std::endl;

                    no_fly_zones_geo_.push_back(actual_no_fly_zone);


                } else if ( std::string(Placemark.child("name").child_value()) == "3" ) {
                    // std::cout << "3 private area" << std::endl;

                    // string that contains the raw data of the edges (coordinates) of the polygon:
                    std::string private_area_string = std::string( Placemark.child("Polygon").child("outerBoundaryIs").child("LinearRing").child("coordinates").child_value() );

                    // Remove new lines if exist in the string:
                    private_area_string.erase(std::remove(private_area_string.begin(), private_area_string.end(), '\n'), private_area_string.end());
                    private_area_string.erase(std::remove(private_area_string.begin(), private_area_string.end(), '\r'), private_area_string.end());

                    std::string::size_type sz;
                    geographic_msgs::GeoPoint private_area_point;
                    std::vector< geographic_msgs::GeoPoint > actual_private_area;

                    // First, erase the blank space before the coordinates (if exist):
                    int i = 0;
                    while ( private_area_string[i]==' ' || private_area_string[i]=='\t' ) {
                        i++;
                        if (i==private_area_string.size()) break;
                    }
                    i--;
                    for ( int j=0; j<=i; j++ ) private_area_string.erase(private_area_string.begin());

                    // Second, erase the blank space after the coordinates (if exist):
                    i = 0;
                    while ( private_area_string[private_area_string.size()-1-i]==' ' || private_area_string[private_area_string.size()-1-i]=='\t' ) {
                        i++;
                        if (i==-1) break;
                    }
                    i--;
                    for ( int j=0; j<=i; j++ ) private_area_string.erase(private_area_string.end()-1);

                    // Third, erase the blank space between pairs of coordinates and insert there a comma ',':
                    i = private_area_string.size()-1;
                    while ( i>0 ) {
                        if ( (private_area_string[i-1]==' ' || private_area_string[i-1]=='\t') && (private_area_string[i]!=' ' && private_area_string[i]!='\t') ) {
                            i--;
                            int j=i;
                            while ( j>0 && (private_area_string[j-1]==' ' || private_area_string[j-1]=='\t') ) j--; //std::cout << i << j << std::endl;}
                            for ( int k=i-j+1; k>0; k-- ) private_area_string.erase(private_area_string.begin()+j);
                            private_area_string.insert(private_area_string.begin()+j,',');
                            i=j-1;
                        }
                        else i--;
                    }

                    // Fourth, reemplace commas ',' with spaces ' ':
                    for ( int j = 0; j<private_area_string.size(); j++ ) {
                        if ( private_area_string[j]==',' ) private_area_string[j]=' ';
                    }

                    // Convert the string data, clean and ready, to double (float64 because of point) with stod.
                    // Each time a number is converted (numbers separated by space ' '), the string is overwriten with the rest of the string using substr.
                    // The quantity of numbers have to be multiple of 3.
                    while ( private_area_string.size()>0 ) {
                        private_area_point.longitude = std::stod (private_area_string,&sz);
                        private_area_string = private_area_string.substr(sz);
                        private_area_point.latitude = std::stod (private_area_string,&sz);
                        private_area_string = private_area_string.substr(sz);
                        private_area_point.altitude = std::stod (private_area_string,&sz);
                        private_area_string = private_area_string.substr(sz);
                        actual_private_area.push_back(private_area_point);
                    }

                    private_areas_geo_.push_back(actual_private_area);


                } else if ( std::string(Placemark.child("name").child_value()) == "4" ) {
                    // std::cout << "4 Potential emergency landing sites" << std::endl;

                    // string that contains the raw data of the edges (coordinates) of the polygon:
                    std::string emergency_landing_site_string = std::string( Placemark.child("Polygon").child("outerBoundaryIs").child("LinearRing").child("coordinates").child_value() );

                    // Remove new lines if exist in the string:
                    emergency_landing_site_string.erase(std::remove(emergency_landing_site_string.begin(), emergency_landing_site_string.end(), '\n'), emergency_landing_site_string.end());
                    emergency_landing_site_string.erase(std::remove(emergency_landing_site_string.begin(), emergency_landing_site_string.end(), '\r'), emergency_landing_site_string.end());

                    std::string::size_type sz;
                    geographic_msgs::GeoPoint emergency_landing_site_point;
                    std::vector< geographic_msgs::GeoPoint > actual_emergency_landing_site;

                    // First, erase the blank space before the coordinates (if exist):
                    int i = 0;
                    while ( emergency_landing_site_string[i]==' ' || emergency_landing_site_string[i]=='\t' ) {
                        i++;
                        if (i==emergency_landing_site_string.size()) break;
                    }
                    i--;
                    for ( int j=0; j<=i; j++ ) emergency_landing_site_string.erase(emergency_landing_site_string.begin());

                    // Second, erase the blank space after the coordinates (if exist):
                    i = 0;
                    while ( emergency_landing_site_string[emergency_landing_site_string.size()-1-i]==' ' || emergency_landing_site_string[emergency_landing_site_string.size()-1-i]=='\t' ) {
                        i++;
                        if (i==-1) break;
                    }
                    i--;
                    for ( int j=0; j<=i; j++ ) emergency_landing_site_string.erase(emergency_landing_site_string.end()-1);

                    // Third, erase the blank space between pairs of coordinates and insert there a comma ',':
                    i = emergency_landing_site_string.size()-1;
                    while ( i>0 ) {
                        if ( (emergency_landing_site_string[i-1]==' ' || emergency_landing_site_string[i-1]=='\t') && (emergency_landing_site_string[i]!=' ' && emergency_landing_site_string[i]!='\t') ) {
                            i--;
                            int j=i;
                            while ( j>0 && (emergency_landing_site_string[j-1]==' ' || emergency_landing_site_string[j-1]=='\t') ) j--; //std::cout << i << j << std::endl;}
                            for ( int k=i-j+1; k>0; k-- ) emergency_landing_site_string.erase(emergency_landing_site_string.begin()+j);
                            emergency_landing_site_string.insert(emergency_landing_site_string.begin()+j,',');
                            i=j-1;
                        } else {
                            i--;
                        }
                    }

                    // Fourth, reemplace commas ',' with spaces ' ':
                    for ( int j = 0; j<emergency_landing_site_string.size(); j++ ) {
                        if ( emergency_landing_site_string[j]==',' ) emergency_landing_site_string[j]=' ';
                    }

                    // Convert the string data, clean and ready, to double (float64 because of point) with stod.
                    // Each time a number is converted (numbers separated by space ' '), the string is overwriten with the rest of the string using substr.
                    // The quantity of numbers have to be multiple of 3.
                    while ( emergency_landing_site_string.size()>0 ) {
                        emergency_landing_site_point.longitude = std::stod (emergency_landing_site_string,&sz);
                        emergency_landing_site_string = emergency_landing_site_string.substr(sz);
                        emergency_landing_site_point.latitude = std::stod (emergency_landing_site_string,&sz);
                        emergency_landing_site_string = emergency_landing_site_string.substr(sz);
                        emergency_landing_site_point.altitude = std::stod (emergency_landing_site_string,&sz);
                        emergency_landing_site_string = emergency_landing_site_string.substr(sz);
                        actual_emergency_landing_site.push_back(emergency_landing_site_point);
                    }

                    emergency_landing_sites_geo_.push_back(actual_emergency_landing_site);


                } else if ( std::string(Placemark.child("name").child_value()) == "5" ) {
                    // std::cout << "5 POI (Point Of Interest)" << std::endl;

                    // string that contains the raw data of coordinates of the point:
                    std::string POI_string = std::string( Placemark.child("Point").child("coordinates").child_value() );

                    // Remove new lines if exist in the string:
                    POI_string.erase(std::remove(POI_string.begin(), POI_string.end(), '\n'), POI_string.end());
                    POI_string.erase(std::remove(POI_string.begin(), POI_string.end(), '\r'), POI_string.end());

                    std::string::size_type sz;
                    geographic_msgs::GeoPoint POI_point;

                    // First, erase the blank spaces after and before the coordinates, converting commas ',' into spaces ' ':
                    for ( int i = POI_string.size()-1; i>=0; i-- ) {
                        if ( POI_string[i]==' ' || POI_string[i]=='\t' ) POI_string.erase(POI_string.begin()+i);
                        else if ( POI_string[i]==',' ) POI_string[i]=' ';
                    }

                    // With the previous string the data is extracted to double (point is in float64) with stod. The string is divided in the first space ' ' with subst:
                    POI_point.longitude = std::stod (POI_string,&sz);
                    POI_string = POI_string.substr(sz);
                    POI_point.latitude = std::stod (POI_string,&sz);
                    POI_point.altitude = std::stod (POI_string.substr(sz));

                    POIs_geo_.push_back(POI_point);


                } else if ( std::string(Placemark.child("name").child_value()) == "6" ) {
                    // std::cout << "6 Road" << std::endl;

                    // string that contains the raw data of the edges (coordinates) of the polygon:
                    std::string road_string = std::string( Placemark.child("Polygon").child("outerBoundaryIs").child("LinearRing").child("coordinates").child_value() );

                    // Remove new lines if exist in the string:
                    road_string.erase(std::remove(road_string.begin(), road_string.end(), '\n'), road_string.end());
                    road_string.erase(std::remove(road_string.begin(), road_string.end(), '\r'), road_string.end());

                    std::string::size_type sz;
                    geographic_msgs::GeoPoint road_point;
                    std::vector< geographic_msgs::GeoPoint > actual_road;

                    // First, erase the blank space before the coordinates (if exist):
                    int i = 0;
                    while ( road_string[i]==' ' || road_string[i]=='\t' ) {
                        i++;
                        if (i==road_string.size()) break;
                    }
                    i--;
                    for ( int j=0; j<=i; j++ ) road_string.erase(road_string.begin());

                    // Second, erase the blank space after the coordinates (if exist):
                    i = 0;
                    while ( road_string[road_string.size()-1-i]==' ' || road_string[road_string.size()-1-i]=='\t' ) {
                        i++;
                        if (i==-1) break;
                    }
                    i--;
                    for ( int j=0; j<=i; j++ ) road_string.erase(road_string.end()-1);

                    // Third, erase the blank space between pairs of coordinates and insert there a comma ',':
                    i = road_string.size()-1;
                    while ( i>0 ) {
                        if ( (road_string[i-1]==' ' || road_string[i-1]=='\t') && (road_string[i]!=' ' && road_string[i]!='\t') ) {
                            i--;
                            int j=i;
                            while ( j>0 && (road_string[j-1]==' ' || road_string[j-1]=='\t') ) j--; //std::cout << i << j << std::endl;}
                            for ( int k=i-j+1; k>0; k-- ) road_string.erase(road_string.begin()+j);
                            road_string.insert(road_string.begin()+j,',');
                            i=j-1;
                        }
                        else i--;
                    }

                    // Fourth, reemplace commas ',' with spaces ' ':
                    for ( int j = 0; j<road_string.size(); j++ ) {
                        if ( road_string[j]==',' ) road_string[j]=' ';
                    }

                    // Convert the string data, clean and ready, to double (float64 because of point) with stod.
                    // Each time a number is converted (numbers separated by space ' '), the string is overwriten with the rest of the string using substr.
                    // The quantity of numbers have to be multiple of 3.
                    while ( road_string.size()>0 ) {
                        road_point.longitude = std::stod (road_string,&sz);
                        road_string = road_string.substr(sz);
                        road_point.latitude = std::stod (road_string,&sz);
                        road_string = road_string.substr(sz);
                        road_point.altitude = std::stod (road_string,&sz);
                        road_string = road_string.substr(sz);
                        actual_road.push_back(road_point);
                    }

                    roads_geo_.push_back(actual_road);


                } else if ( std::string(Placemark.child("name").child_value()) == "7" ) {
                    // std::cout << "7 River-Lake" << std::endl;

                    // string that contains the raw data of the edges (coordinates) of the polygon:
                    std::string river_or_lake_string = std::string( Placemark.child("Polygon").child("outerBoundaryIs").child("LinearRing").child("coordinates").child_value() );

                    // Remove new lines if exist in the string:
                    river_or_lake_string.erase(std::remove(river_or_lake_string.begin(), river_or_lake_string.end(), '\n'), river_or_lake_string.end());
                    river_or_lake_string.erase(std::remove(river_or_lake_string.begin(), river_or_lake_string.end(), '\r'), river_or_lake_string.end());

                    std::string::size_type sz;
                    geographic_msgs::GeoPoint river_or_lake_point;
                    std::vector< geographic_msgs::GeoPoint > actual_river_or_lake;

                    // First, erase the blank space before the coordinates (if exist):
                    int i = 0;
                    while ( river_or_lake_string[i]==' ' || river_or_lake_string[i]=='\t' ) {
                        i++;
                        if (i==river_or_lake_string.size()) break;
                    }
                    i--;
                    for ( int j=0; j<=i; j++ ) river_or_lake_string.erase(river_or_lake_string.begin());

                    // Second, erase the blank space after the coordinates (if exist):
                    i = 0;
                    while ( river_or_lake_string[river_or_lake_string.size()-1-i]==' ' || river_or_lake_string[river_or_lake_string.size()-1-i]=='\t' ) {
                        i++;
                        if (i==-1) break;
                    }
                    i--;
                    for ( int j=0; j<=i; j++ ) river_or_lake_string.erase(river_or_lake_string.end()-1);

                    // Third, erase the blank space between pairs of coordinates and insert there a comma ',':
                    i = river_or_lake_string.size()-1;
                    while ( i>0 ) {
                        if ( (river_or_lake_string[i-1]==' ' || river_or_lake_string[i-1]=='\t') && (river_or_lake_string[i]!=' ' && river_or_lake_string[i]!='\t') ) {
                            i--;
                            int j=i;
                            while ( j>0 && (river_or_lake_string[j-1]==' ' || river_or_lake_string[j-1]=='\t') ) j--; //std::cout << i << j << std::endl;}
                            for ( int k=i-j+1; k>0; k-- ) river_or_lake_string.erase(river_or_lake_string.begin()+j);
                            river_or_lake_string.insert(river_or_lake_string.begin()+j,',');
                            i=j-1;
                        } else {
                            i--;
                        }
                    }

                    // Fourth, reemplace commas ',' with spaces ' ':
                    for ( int j = 0; j<river_or_lake_string.size(); j++ ) {
                        if ( river_or_lake_string[j]==',' ) river_or_lake_string[j]=' ';
                    }

                    // Convert the string data, clean and ready, to double (float64 because of point) with stod.
                    // Each time a number is converted (numbers separated by space ' '), the string is overwriten with the rest of the string using substr.
                    // The quantity of numbers have to be multiple of 3.
                    while ( river_or_lake_string.size()>0 ) {
                        river_or_lake_point.longitude = std::stod (river_or_lake_string,&sz);
                        river_or_lake_string = river_or_lake_string.substr(sz);
                        river_or_lake_point.latitude = std::stod (river_or_lake_string,&sz);
                        river_or_lake_string = river_or_lake_string.substr(sz);
                        river_or_lake_point.altitude = std::stod (river_or_lake_string,&sz);
                        river_or_lake_string = river_or_lake_string.substr(sz);
                        actual_river_or_lake.push_back(river_or_lake_point);
                    }

                    rivers_or_lakes_geo_.push_back(actual_river_or_lake);


                } else if ( std::string(Placemark.child("name").child_value()) == "8" ) {
                    // std::cout << "8 stadium" << std::endl;

                    // string that contains the raw data of the edges (coordinates) of the polygon:
                    std::string stadium_string = std::string( Placemark.child("Polygon").child("outerBoundaryIs").child("LinearRing").child("coordinates").child_value() );

                    // Remove new lines if exist in the string:
                    stadium_string.erase(std::remove(stadium_string.begin(), stadium_string.end(), '\n'), stadium_string.end());
                    stadium_string.erase(std::remove(stadium_string.begin(), stadium_string.end(), '\r'), stadium_string.end());

                    std::string::size_type sz;
                    geographic_msgs::GeoPoint stadium_point;
                    std::vector< geographic_msgs::GeoPoint > actual_stadium;

                    // First, erase the blank space before the coordinates (if exist):
                    int i = 0;
                    while ( stadium_string[i]==' ' || stadium_string[i]=='\t' ) {
                        i++;
                        if (i==stadium_string.size()) break;
                    }
                    i--;
                    for ( int j=0; j<=i; j++ ) stadium_string.erase(stadium_string.begin());

                    // Second, erase the blank space after the coordinates (if exist):
                    i = 0;
                    while ( stadium_string[stadium_string.size()-1-i]==' ' || stadium_string[stadium_string.size()-1-i]=='\t' ) {
                        i++;
                        if (i==-1) break;
                    }
                    i--;
                    for ( int j=0; j<=i; j++ ) stadium_string.erase(stadium_string.end()-1);

                    // Third, erase the blank space between pairs of coordinates and insert there a comma ',':
                    i = stadium_string.size()-1;
                    while ( i>0 ) {
                        if ( (stadium_string[i-1]==' ' || stadium_string[i-1]=='\t') && (stadium_string[i]!=' ' && stadium_string[i]!='\t') ) {
                            i--;
                            int j=i;
                            while ( j>0 && (stadium_string[j-1]==' ' || stadium_string[j-1]=='\t') ) j--; //std::cout << i << j << std::endl;}
                            for ( int k=i-j+1; k>0; k-- ) stadium_string.erase(stadium_string.begin()+j);
                            stadium_string.insert(stadium_string.begin()+j,',');
                            i=j-1;
                        }
                        else {
                            i--;
                        }
                    }

                    // Fourth, reemplace commas ',' with spaces ' ':
                    for ( int j = 0; j<stadium_string.size(); j++ ) {
                        if ( stadium_string[j]==',' ) stadium_string[j]=' ';
                    }

                    // Convert the string data, clean and ready, to double (float64 because of point) with stod.
                    // Each time a number is converted (numbers separated by space ' '), the string is overwriten with the rest of the string using substr.
                    // The quantity of numbers have to be multiple of 3.
                    while ( stadium_string.size()>0 ) {
                        stadium_point.longitude = std::stod (stadium_string,&sz);
                        stadium_string = stadium_string.substr(sz);
                        stadium_point.latitude = std::stod (stadium_string,&sz);
                        stadium_string = stadium_string.substr(sz);
                        stadium_point.altitude = std::stod (stadium_string,&sz);
                        stadium_string = stadium_string.substr(sz);
                        actual_stadium.push_back(stadium_point);
                    }

                    stadiums_geo_.push_back(actual_stadium);


                } else if ( std::string(Placemark.child("name").child_value()) == "9" ) {
                    // std::cout << "9 Reference Target Starting Point" << std::endl;

                    // string that contains the raw data of coordinates of the point:
                    std::string reference_target_start_string = std::string( Placemark.child("Point").child("coordinates").child_value() );

                    // Remove new lines if exist in the string:
                    reference_target_start_string.erase(std::remove(reference_target_start_string.begin(), reference_target_start_string.end(), '\n'), reference_target_start_string.end());
                    reference_target_start_string.erase(std::remove(reference_target_start_string.begin(), reference_target_start_string.end(), '\r'), reference_target_start_string.end());

                    std::string::size_type sz;
                    geographic_msgs::GeoPoint reference_target_start_point;

                    // First, erase the blank spaces after and before the coordinates, converting commas ',' into spaces ' ':
                    for ( int i = reference_target_start_string.size()-1; i>=0; i-- ) {
                        if ( reference_target_start_string[i]==' ' || reference_target_start_string[i]=='\t' ) reference_target_start_string.erase(reference_target_start_string.begin()+i);
                        else if ( reference_target_start_string[i]==',' ) reference_target_start_string[i]=' ';
                    }

                    // With the previous string the data is extracted to double (point is in float64) with stod. The string is divided in the first space ' ' with subst:
                    reference_target_start_point.longitude = std::stod (reference_target_start_string,&sz);
                    reference_target_start_string = reference_target_start_string.substr(sz);
                    reference_target_start_point.latitude = std::stod (reference_target_start_string,&sz);
                    reference_target_start_point.altitude = std::stod (reference_target_start_string.substr(sz));

                    origin_of_formation_geo_.push_back(reference_target_start_point);


                } else if ( std::string(Placemark.child("name").child_value()) == "10" ) {
                    // std::cout << "10 Reference Target Trajectory" << std::endl;

                    // string that contains the raw data of the edges (coordinates) of the polygon:
                    std::string reference_target_trajectory_string = std::string( Placemark.child("LineString").child("coordinates").child_value() );

                    // Remove new lines if exist in the string:
                    reference_target_trajectory_string.erase(std::remove(reference_target_trajectory_string.begin(), reference_target_trajectory_string.end(), '\n'), reference_target_trajectory_string.end());
                    reference_target_trajectory_string.erase(std::remove(reference_target_trajectory_string.begin(), reference_target_trajectory_string.end(), '\r'), reference_target_trajectory_string.end());

                    std::string::size_type sz;
                    geographic_msgs::GeoPoint reference_target_trajectory_point;
                    std::vector< geographic_msgs::GeoPoint > actual_reference_target_trajectory;

                    // First, erase the blank space before the coordinates (if exist):
                    int i = 0;
                    while ( reference_target_trajectory_string[i]==' ' || reference_target_trajectory_string[i]=='\t' ) {
                        i++;
                        if (i==reference_target_trajectory_string.size()) break;
                    }
                    i--;
                    for ( int j=0; j<=i; j++ ) reference_target_trajectory_string.erase(reference_target_trajectory_string.begin());

                    // Second, erase the blank space after the coordinates (if exist):
                    i = 0;
                    while ( reference_target_trajectory_string[reference_target_trajectory_string.size()-1-i]==' ' || reference_target_trajectory_string[reference_target_trajectory_string.size()-1-i]=='\t' ) {
                        i++;
                        if (i==-1) break;
                    }
                    i--;
                    for ( int j=0; j<=i; j++ ) reference_target_trajectory_string.erase(reference_target_trajectory_string.end()-1);

                    // Third, erase the blank space between pairs of coordinates and insert there a comma ',':
                    i = reference_target_trajectory_string.size()-1;
                    while ( i>0 ) {
                        if ( (reference_target_trajectory_string[i-1]==' ' || reference_target_trajectory_string[i-1]=='\t') && (reference_target_trajectory_string[i]!=' ' && reference_target_trajectory_string[i]!='\t') ) {
                            i--;
                            int j=i;
                            while ( j>0 && (reference_target_trajectory_string[j-1]==' ' || reference_target_trajectory_string[j-1]=='\t') ) j--; //std::cout << i << j << std::endl;}
                            for ( int k=i-j+1; k>0; k-- ) reference_target_trajectory_string.erase(reference_target_trajectory_string.begin()+j);
                            reference_target_trajectory_string.insert(reference_target_trajectory_string.begin()+j,',');
                            i=j-1;
                        }
                        else {
                            i--;
                        }
                    }

                    // Fourth, reemplace commas ',' with spaces ' ':
                    for ( int j = 0; j<reference_target_trajectory_string.size(); j++ ) {
                        if ( reference_target_trajectory_string[j]==',' ) reference_target_trajectory_string[j]=' ';
                    }

                    // Convert the string data, clean and ready, to double (float64 because of point) with stod.
                    // Each time a number is converted (numbers separated by space ' '), the string is overwriten with the rest of the string using substr.
                    // The quantity of numbers have to be multiple of 3.
                    while ( reference_target_trajectory_string.size()>0 ) {
                        reference_target_trajectory_point.longitude = std::stod (reference_target_trajectory_string,&sz);
                        reference_target_trajectory_string = reference_target_trajectory_string.substr(sz);
                        reference_target_trajectory_point.latitude = std::stod (reference_target_trajectory_string,&sz);
                        reference_target_trajectory_string = reference_target_trajectory_string.substr(sz);
                        reference_target_trajectory_point.altitude = std::stod (reference_target_trajectory_string,&sz);
                        reference_target_trajectory_string = reference_target_trajectory_string.substr(sz);
                        actual_reference_target_trajectory.push_back(reference_target_trajectory_point);
                    }

                    reference_targets_trajectories_geo_.push_back(actual_reference_target_trajectory);
                } else if ( std::string(Placemark.child("name").child_value()) == "11" ) {
                    // std::cout << "11 geofencing" << std::endl;

                    // string that contains the raw data of the edges (coordinates) of the polygon:
                    std::string geofencing_string = std::string( Placemark.child("Polygon").child("outerBoundaryIs").child("LinearRing").child("coordinates").child_value() );

                    // Remove new lines if exist in the string:
                    geofencing_string.erase(std::remove(geofencing_string.begin(), geofencing_string.end(), '\n'), geofencing_string.end());
                    geofencing_string.erase(std::remove(geofencing_string.begin(), geofencing_string.end(), '\r'), geofencing_string.end());

                    std::string::size_type sz;
                    geographic_msgs::GeoPoint geofencing_point;
                    std::vector< geographic_msgs::GeoPoint > actual_geofencing;

                    // First, erase the blank space before the coordinates (if exist):
                    int i = 0;
                    while ( geofencing_string[i]==' ' || geofencing_string[i]=='\t' ) {
                        i++;
                        if (i==geofencing_string.size()) break;
                    }
                    i--;
                    for ( int j=0; j<=i; j++ ) geofencing_string.erase(geofencing_string.begin());

                    // Second, erase the blank space after the coordinates (if exist):
                    i = 0;
                    while ( geofencing_string[geofencing_string.size()-1-i]==' ' || geofencing_string[geofencing_string.size()-1-i]=='\t' ) {
                        i++;
                        if (i==-1) break;
                    }
                    i--;
                    for ( int j=0; j<=i; j++ ) geofencing_string.erase(geofencing_string.end()-1);

                    // Third, erase the blank space between pairs of coordinates and insert there a comma ',':
                    i = geofencing_string.size()-1;
                    while ( i>0 ) {
                        if ( (geofencing_string[i-1]==' ' || geofencing_string[i-1]=='\t') && (geofencing_string[i]!=' ' && geofencing_string[i]!='\t') ) {
                            i--;
                            int j=i;
                            while ( j>0 && (geofencing_string[j-1]==' ' || geofencing_string[j-1]=='\t') ) j--; //std::cout << i << j << std::endl;}
                            for ( int k=i-j+1; k>0; k-- ) geofencing_string.erase(geofencing_string.begin()+j);
                            geofencing_string.insert(geofencing_string.begin()+j,',');
                            i=j-1;
                        } else {
                            i--;
                        }
                    }

                    // Fourth, reemplace commas ',' with spaces ' ':
                    for ( int j = 0; j<geofencing_string.size(); j++ ) {
                        if ( geofencing_string[j]==',' ) geofencing_string[j]=' ';
                    }

                    // // DEBUG, show the string with the clean data, ready for the double conversion
                    // std::cout << geofencing_string << std::endl;

                    // Convert the string data, clean and ready, to double (float64 because of point) with stod.
                    // Each time a number is converted (numbers separated by space ' '), the string is overwriten with the rest of the string using substr.
                    // The quantity of numbers have to be multiple of 3.
                    while ( geofencing_string.size()>0 ) {
                        geofencing_point.longitude = std::stod (geofencing_string,&sz);
                        geofencing_string = geofencing_string.substr(sz);
                        geofencing_point.latitude = std::stod (geofencing_string,&sz);
                        geofencing_string = geofencing_string.substr(sz);
                        geofencing_point.altitude = std::stod (geofencing_string,&sz);
                        geofencing_string = geofencing_string.substr(sz);
                        actual_geofencing.push_back(geofencing_point);
                    }

                    // // DEBUG, show the whole actual_geofencing vector of points
                    // for (int j=0; j<actual_geofencing.size(); j++) std::cout << std::setprecision (10) <<actual_geofencing[j];
                    // std::cout << std::endl;

                    geofence_geo_ = actual_geofencing;    // Only one geofencing expected, replace the previous if exist.


                }

            }
        }

        // ///////////// DEBUG, show the results after the KML parser. /////////////
        // std::cout << "stations_geo_" << std::endl;                       for (int i=0; i<stations_geo_.size(); i++)                                                                                            std::cout << std::setprecision (10) << stations_geo_[i];
        // std::cout << "no_fly_zones_geo_" << std::endl;                   for (int i=0; i<no_fly_zones_geo_.size(); i++)                    for (int j=0; j<no_fly_zones_geo_[i].size(); j++)                   std::cout << std::setprecision (10) << no_fly_zones_geo_[i][j];
        // std::cout << "private_areas_geo_" << std::endl;                  for (int i=0; i<private_areas_geo_.size(); i++)                   for (int j=0; j<private_areas_geo_[i].size(); j++)                  std::cout << std::setprecision (10) << private_areas_geo_[i][j];
        // std::cout << "emergency_landing_sites_geo_" << std::endl;        for (int i=0; i<emergency_landing_sites_geo_.size(); i++)         for (int j=0; j<emergency_landing_sites_geo_[i].size(); j++)        std::cout << std::setprecision (10) << emergency_landing_sites_geo_[i][j];
        // std::cout << "POIs_geo_" << std::endl;                           for (int i=0; i<POIs_geo_.size(); i++)                                                                                                std::cout << std::setprecision (10) << POIs_geo_[i];
        // std::cout << "roads_geo_" << std::endl;                          for (int i=0; i<roads_geo_.size(); i++)                           for (int j=0; j<roads_geo_[i].size(); j++)                          std::cout << std::setprecision (10) << roads_geo_[i][j];
        // std::cout << "rivers_or_lakes_geo_" << std::endl;                for (int i=0; i<rivers_or_lakes_geo_.size(); i++)                 for (int j=0; j<rivers_or_lakes_geo_[i].size(); j++)                std::cout << std::setprecision (10) << rivers_or_lakes_geo_[i][j];
        // std::cout << "stadiums_geo_" << std::endl;                       for (int i=0; i<stadiums_geo_.size(); i++)                        for (int j=0; j<stadiums_geo_[i].size(); j++)                       std::cout << std::setprecision (10) << stadiums_geo_[i][j];
        // std::cout << "origin_of_formation_geo_" << std::endl;            for (int i=0; i<origin_of_formation_geo_.size(); i++)                                                                                 std::cout << std::setprecision (10) << origin_of_formation_geo_[i];
        // std::cout << "reference_targets_trajectories_geo_" << std::endl; for (int i=0; i<reference_targets_trajectories_geo_.size(); i++)  for (int j=0; j<reference_targets_trajectories_geo_[i].size(); j++) std::cout << std::setprecision (10) << reference_targets_trajectories_geo_[i][j];
        // std::cout << "geofence_geo_" << std::endl;                     for (int i=0; i<geofence_geo_.size(); i++)                                                                                       std::cout << std::setprecision (10) << geofence_geo_[i];
        // ///////////// DEBUG, show the results after the KML parser. /////////////




        // Up until now, "runParser" method of KML parser have fill the "geo_" attributes in geographic coordinates (longitude, latitude, altitude). Cartesian coordinates are needed, so the first thing to do is transform geographic into UTM:
        std::vector< geodesy::UTMPoint >                 stations_UTM;
        std::vector< std::vector<geodesy::UTMPoint> >    no_fly_zones_UTM;
        std::vector< std::vector<geodesy::UTMPoint> >    private_areas_UTM;
        std::vector< std::vector<geodesy::UTMPoint> >    emergency_landing_sites_UTM;
        std::vector< geodesy::UTMPoint >                 POIs_UTM;
        std::vector< std::vector<geodesy::UTMPoint> >    roads_UTM;
        std::vector< std::vector<geodesy::UTMPoint> >    rivers_or_lakes_UTM;
        std::vector< std::vector<geodesy::UTMPoint> >    stadiums_UTM;
        std::vector< geodesy::UTMPoint >                 origin_of_formation_UTM;
        std::vector< std::vector<geodesy::UTMPoint> >    reference_targets_trajectories_UTM;
        std::vector<geodesy::UTMPoint>                   geofencing_UTM;

        for (int i=0; i<stations_geo_.size(); i++) {
            geodesy::UTMPoint utm_pt(stations_geo_[i]);
            stations_UTM.push_back(utm_pt);
        }
        for (int i=0; i<no_fly_zones_geo_.size(); i++) {
            std::vector<geodesy::UTMPoint> actual_no_fly_zone_UTM;
            for (int j=0; j<no_fly_zones_geo_[i].size(); j++) {
                geodesy::UTMPoint utm_pt(no_fly_zones_geo_[i][j]);
                actual_no_fly_zone_UTM.push_back(utm_pt);
            }
            no_fly_zones_UTM.push_back(actual_no_fly_zone_UTM);
        }
        for (int i=0; i<private_areas_geo_.size(); i++) {
            std::vector<geodesy::UTMPoint> actual_private_area_UTM;
            for (int j=0; j<private_areas_geo_[i].size(); j++) {
                geodesy::UTMPoint utm_pt(private_areas_geo_[i][j]);
                actual_private_area_UTM.push_back(utm_pt);
            }
            private_areas_UTM.push_back(actual_private_area_UTM);
        }
        for (int i=0; i<emergency_landing_sites_geo_.size(); i++) {
            std::vector<geodesy::UTMPoint> actual_emergency_landing_site_UTM;
            for (int j=0; j<emergency_landing_sites_geo_[i].size(); j++) {
                geodesy::UTMPoint utm_pt(emergency_landing_sites_geo_[i][j]);
                actual_emergency_landing_site_UTM.push_back(utm_pt);
            }
            emergency_landing_sites_UTM.push_back(actual_emergency_landing_site_UTM);
        }
        for (int i=0; i<POIs_geo_.size(); i++) {
            geodesy::UTMPoint utm_pt(POIs_geo_[i]);
            POIs_UTM.push_back(utm_pt);
        }
        for (int i=0; i<roads_geo_.size(); i++) {
            std::vector<geodesy::UTMPoint> actual_road_UTM;
            for (int j=0; j<roads_geo_[i].size(); j++) {
                geodesy::UTMPoint utm_pt(roads_geo_[i][j]);
                actual_road_UTM.push_back(utm_pt);
            }
            roads_UTM.push_back(actual_road_UTM);
        }
        for (int i=0; i<rivers_or_lakes_geo_.size(); i++) {
            std::vector<geodesy::UTMPoint> actual_river_or_lake_UTM;
            for (int j=0; j<rivers_or_lakes_geo_[i].size(); j++) {
                geodesy::UTMPoint utm_pt(rivers_or_lakes_geo_[i][j]);
                actual_river_or_lake_UTM.push_back(utm_pt);
            }
            rivers_or_lakes_UTM.push_back(actual_river_or_lake_UTM);
        }
        for (int i=0; i<stadiums_geo_.size(); i++) {
            std::vector<geodesy::UTMPoint> actual_stadium_UTM;
            for (int j=0; j<stadiums_geo_[i].size(); j++) {
                geodesy::UTMPoint utm_pt(stadiums_geo_[i][j]);
                actual_stadium_UTM.push_back(utm_pt);
                }
            stadiums_UTM.push_back(actual_stadium_UTM);
        }
        for (int i=0; i<origin_of_formation_geo_.size(); i++) {
            geodesy::UTMPoint utm_pt(origin_of_formation_geo_[i]);
            origin_of_formation_UTM.push_back(utm_pt);
        }
        for (int i=0; i<reference_targets_trajectories_geo_.size(); i++) {
            std::vector<geodesy::UTMPoint> actual_reference_target_trajectory_UTM;
            for (int j=0; j<reference_targets_trajectories_geo_[i].size(); j++) {
                geodesy::UTMPoint utm_pt(reference_targets_trajectories_geo_[i][j]);
                actual_reference_target_trajectory_UTM.push_back(utm_pt);
            }
            reference_targets_trajectories_UTM.push_back(actual_reference_target_trajectory_UTM);
        }
        for (int i=0; i<geofence_geo_.size(); i++) {
            geodesy::UTMPoint utm_pt(geofence_geo_[i]);
            geofencing_UTM.push_back(utm_pt);
        }

        // ///////////// DEBUG, show the results after converting coordinates to UTM. /////////////
        // std::cout << "stations_UTM" << std::endl;                       for (int i=0; i<stations_UTM.size(); i++)                                                                                           std::cout << std::setprecision (10) << stations_UTM[i] << std::endl;
        // std::cout << "no_fly_zones_UTM" << std::endl;                   for (int i=0; i<no_fly_zones_UTM.size(); i++)                    for (int j=0; j<no_fly_zones_UTM[i].size(); j++)                   std::cout << std::setprecision (10) << no_fly_zones_UTM[i][j] << std::endl;
        // std::cout << "private_areas_UTM" << std::endl;                  for (int i=0; i<private_areas_UTM.size(); i++)                   for (int j=0; j<private_areas_UTM[i].size(); j++)                  std::cout << std::setprecision (10) << private_areas_UTM[i][j] << std::endl;
        // std::cout << "emergency_landing_sites_UTM" << std::endl;        for (int i=0; i<emergency_landing_sites_UTM.size(); i++)         for (int j=0; j<emergency_landing_sites_UTM[i].size(); j++)        std::cout << std::setprecision (10) << emergency_landing_sites_UTM[i][j] << std::endl;
        // std::cout << "POIs_UTM" << std::endl;                           for (int i=0; i<POIs_UTM.size(); i++)                                                                                               std::cout << std::setprecision (10) << POIs_UTM[i] << std::endl;
        // std::cout << "roads_UTM" << std::endl;                          for (int i=0; i<roads_UTM.size(); i++)                           for (int j=0; j<roads_UTM[i].size(); j++)                          std::cout << std::setprecision (10) << roads_UTM[i][j] << std::endl;
        // std::cout << "rivers_or_lakes_UTM" << std::endl;                for (int i=0; i<rivers_or_lakes_UTM.size(); i++)                 for (int j=0; j<rivers_or_lakes_UTM[i].size(); j++)                std::cout << std::setprecision (10) << rivers_or_lakes_UTM[i][j] << std::endl;
        // std::cout << "stadiums_UTM" << std::endl;                       for (int i=0; i<stadiums_UTM.size(); i++)                        for (int j=0; j<stadiums_UTM[i].size(); j++)                       std::cout << std::setprecision (10) << stadiums_UTM[i][j] << std::endl;
        // std::cout << "origin_of_formation_UTM" << std::endl;            for (int i=0; i<origin_of_formation_UTM.size(); i++)                                                                                std::cout << std::setprecision (10) << origin_of_formation_UTM[i] << std::endl;
        // std::cout << "reference_targets_trajectories_UTM" << std::endl; for (int i=0; i<reference_targets_trajectories_UTM.size(); i++)  for (int j=0; j<reference_targets_trajectories_UTM[i].size(); j++) std::cout << std::setprecision (10) << reference_targets_trajectories_UTM[i][j] << std::endl;
        // std::cout << "geofencing_UTM" << std::endl;                     for (int i=0; i<geofencing_UTM.size(); i++)                                                                                      std::cout << std::setprecision (10) << geofencing_UTM[i] << std::endl;
        // ///////////// DEBUG, show the results after converting coordinates to UTM. /////////////




        geodesy::UTMPoint origin_coordinate_UTM(_origin_coordinates_geo);


        // The problem with UTM is that if there are coordinates in more than one zone, it's difficult to merge the coordinates of the different zones.
        // This is because each zone has 6 degrees of longitude, and the width in meters of the zone is variable from equator to the poles.
        // Something similar when the coordinates are in different hemispheres, separated by the equator. Each hemisphere has a different origin for the y axis.
        // These are the reasons why the x and y assignation for the Cartesian conversion isn't a simple UTM substraction (see "UTM_to_cartesian.hpp" or "geographic_to_cartesian.hpp" libraries for more detail).

        min_x_ = std::numeric_limits<double>::max() ;    // Minimum coordinate initialized to "infinity" (maximum value possible for double).
        max_x_ = -std::numeric_limits<double>::max() ;   // Maximum coordinate initialized to minus "infinity" (minimum value possible for double).
        min_y_ = std::numeric_limits<double>::max() ;    // Minimum coordinate initialized to "infinity" (maximum value possible for double).
        max_y_ = -std::numeric_limits<double>::max() ;   // Maximum coordinate initialized to minus "infinity" (minimum value possible for double).

        for (int i=0; i<stations_UTM.size(); i++) {
            geometry_msgs::Point32 actual_coordinate_cartesian = UTM_to_cartesian (stations_UTM[i], stations_geo_[i], origin_coordinate_UTM, _origin_coordinates_geo);
            stations_cartesian_.push_back(actual_coordinate_cartesian);
            if (actual_coordinate_cartesian.x < min_x_)   min_x_ = actual_coordinate_cartesian.x;
            if (actual_coordinate_cartesian.x > max_x_)   max_x_ = actual_coordinate_cartesian.x;
            if (actual_coordinate_cartesian.y < min_y_)   min_y_ = actual_coordinate_cartesian.y;
            if (actual_coordinate_cartesian.y > max_y_)   max_y_ = actual_coordinate_cartesian.y;

            // // This is to check if the library geographic_to_cartesian.hpp works. This library transform to Cartesian directly from geographic points. This won't be used because the UTM points can't be checked in debug, but works perfectly.
            // geometry_msgs::Point32 actual_coordinate_cartesian_from_alternative_library = geographic_to_cartesian(stations_geo_[i], stations_geo_[0]);
            // std::cout << actual_coordinate_cartesian << std::endl;
            // std::cout << actual_coordinate_cartesian_from_alternative_library << std::endl;
            // if ( not( (actual_coordinate_cartesian.x == actual_coordinate_cartesian_from_alternative_library.x) && (actual_coordinate_cartesian.y == actual_coordinate_cartesian_from_alternative_library.y) && (actual_coordinate_cartesian.z == actual_coordinate_cartesian_from_alternative_library.z) ) )
            //     std::cout << "The alternative library didn't work well." << std::endl;
        }
        for (int i=0; i<no_fly_zones_UTM.size(); i++) {
            std::vector<geometry_msgs::Point32> actual_no_fly_zone_cartesian;
            for (int j=0; j<no_fly_zones_UTM[i].size(); j++) {
                geometry_msgs::Point32 actual_coordinate_cartesian = UTM_to_cartesian (no_fly_zones_UTM[i][j], no_fly_zones_geo_[i][j], origin_coordinate_UTM, _origin_coordinates_geo);
                actual_no_fly_zone_cartesian.push_back(actual_coordinate_cartesian);
                if (actual_coordinate_cartesian.x < min_x_)   min_x_ = actual_coordinate_cartesian.x;
                if (actual_coordinate_cartesian.x > max_x_)   max_x_ = actual_coordinate_cartesian.x;
                if (actual_coordinate_cartesian.y < min_y_)   min_y_ = actual_coordinate_cartesian.y;
                if (actual_coordinate_cartesian.y > max_y_)   max_y_ = actual_coordinate_cartesian.y;
            }
            no_fly_zones_cartesian_.push_back(actual_no_fly_zone_cartesian);
        }
        for (int i=0; i<private_areas_UTM.size(); i++) {
            std::vector<geometry_msgs::Point32> actual_private_area_cartesian;
            for (int j=0; j<private_areas_UTM[i].size(); j++) {
                geometry_msgs::Point32 actual_coordinate_cartesian = UTM_to_cartesian (private_areas_UTM[i][j], private_areas_geo_[i][j], origin_coordinate_UTM, _origin_coordinates_geo);
                actual_private_area_cartesian.push_back(actual_coordinate_cartesian);
                if (actual_coordinate_cartesian.x < min_x_)   min_x_ = actual_coordinate_cartesian.x;
                if (actual_coordinate_cartesian.x > max_x_)   max_x_ = actual_coordinate_cartesian.x;
                if (actual_coordinate_cartesian.y < min_y_)   min_y_ = actual_coordinate_cartesian.y;
                if (actual_coordinate_cartesian.y > max_y_)   max_y_ = actual_coordinate_cartesian.y;
            }
            private_areas_cartesian_.push_back(actual_private_area_cartesian);
        }
        for (int i=0; i<emergency_landing_sites_UTM.size(); i++) {
            std::vector<geometry_msgs::Point32> actual_emergency_landing_site_cartesian;
            for (int j=0; j<emergency_landing_sites_UTM[i].size(); j++) {
                geometry_msgs::Point32 actual_coordinate_cartesian = UTM_to_cartesian (emergency_landing_sites_UTM[i][j], emergency_landing_sites_geo_[i][j], origin_coordinate_UTM, _origin_coordinates_geo);
                actual_emergency_landing_site_cartesian.push_back(actual_coordinate_cartesian);
                if (actual_coordinate_cartesian.x < min_x_)   min_x_ = actual_coordinate_cartesian.x;
                if (actual_coordinate_cartesian.x > max_x_)   max_x_ = actual_coordinate_cartesian.x;
                if (actual_coordinate_cartesian.y < min_y_)   min_y_ = actual_coordinate_cartesian.y;
                if (actual_coordinate_cartesian.y > max_y_)   max_y_ = actual_coordinate_cartesian.y;
            }
            emergency_landing_sites_cartesian_.push_back(actual_emergency_landing_site_cartesian);
        }
        for (int i=0; i<POIs_UTM.size(); i++) {
            geometry_msgs::Point32 actual_coordinate_cartesian = UTM_to_cartesian (POIs_UTM[i], POIs_geo_[i], origin_coordinate_UTM, _origin_coordinates_geo);
            POIs_cartesian_.push_back(actual_coordinate_cartesian);
            if (actual_coordinate_cartesian.x < min_x_)   min_x_ = actual_coordinate_cartesian.x;
            if (actual_coordinate_cartesian.x > max_x_)   max_x_ = actual_coordinate_cartesian.x;
            if (actual_coordinate_cartesian.y < min_y_)   min_y_ = actual_coordinate_cartesian.y;
            if (actual_coordinate_cartesian.y > max_y_)   max_y_ = actual_coordinate_cartesian.y;
        }
        for (int i=0; i<roads_UTM.size(); i++) {
            std::vector<geometry_msgs::Point32> actual_road_cartesian;
            for (int j=0; j<roads_UTM[i].size(); j++) {
                geometry_msgs::Point32 actual_coordinate_cartesian = UTM_to_cartesian (roads_UTM[i][j], roads_geo_[i][j], origin_coordinate_UTM, _origin_coordinates_geo);
                actual_road_cartesian.push_back(actual_coordinate_cartesian);
                if (actual_coordinate_cartesian.x < min_x_)   min_x_ = actual_coordinate_cartesian.x;
                if (actual_coordinate_cartesian.x > max_x_)   max_x_ = actual_coordinate_cartesian.x;
                if (actual_coordinate_cartesian.y < min_y_)   min_y_ = actual_coordinate_cartesian.y;
                if (actual_coordinate_cartesian.y > max_y_)   max_y_ = actual_coordinate_cartesian.y;
            }
            roads_cartesian_.push_back(actual_road_cartesian);
        }
        for (int i=0; i<rivers_or_lakes_UTM.size(); i++) {
            std::vector<geometry_msgs::Point32> actual_river_or_lake;
            for (int j=0; j<rivers_or_lakes_UTM[i].size(); j++) {
                geometry_msgs::Point32 actual_coordinate_cartesian = UTM_to_cartesian (rivers_or_lakes_UTM[i][j], rivers_or_lakes_geo_[i][j], origin_coordinate_UTM, _origin_coordinates_geo);
                actual_river_or_lake.push_back(actual_coordinate_cartesian);
                if (actual_coordinate_cartesian.x < min_x_)   min_x_ = actual_coordinate_cartesian.x;
                if (actual_coordinate_cartesian.x > max_x_)   max_x_ = actual_coordinate_cartesian.x;
                if (actual_coordinate_cartesian.y < min_y_)   min_y_ = actual_coordinate_cartesian.y;
                if (actual_coordinate_cartesian.y > max_y_)   max_y_ = actual_coordinate_cartesian.y;
            }
            rivers_or_lakes_cartesian_.push_back(actual_river_or_lake);
        }
        for (int i=0; i<stadiums_UTM.size(); i++) {
            std::vector<geometry_msgs::Point32> actual_stadium_cartesian;
            for (int j=0; j<stadiums_UTM[i].size(); j++) {
                geometry_msgs::Point32 actual_coordinate_cartesian = UTM_to_cartesian (stadiums_UTM[i][j], stadiums_geo_[i][j], origin_coordinate_UTM, _origin_coordinates_geo);
                actual_stadium_cartesian.push_back(actual_coordinate_cartesian);
                if (actual_coordinate_cartesian.x < min_x_)   min_x_ = actual_coordinate_cartesian.x;
                if (actual_coordinate_cartesian.x > max_x_)   max_x_ = actual_coordinate_cartesian.x;
                if (actual_coordinate_cartesian.y < min_y_)   min_y_ = actual_coordinate_cartesian.y;
                if (actual_coordinate_cartesian.y > max_y_)   max_y_ = actual_coordinate_cartesian.y;
            }
            stadiums_cartesian_.push_back(actual_stadium_cartesian);
        }
        for (int i=0; i<origin_of_formation_UTM.size(); i++) {
            geometry_msgs::Point32 actual_coordinate_cartesian = UTM_to_cartesian (origin_of_formation_UTM[i], origin_of_formation_geo_[i], origin_coordinate_UTM, _origin_coordinates_geo);
            origin_of_formation_cartesian_.push_back(actual_coordinate_cartesian);
            if (actual_coordinate_cartesian.x < min_x_)   min_x_ = actual_coordinate_cartesian.x;
            if (actual_coordinate_cartesian.x > max_x_)   max_x_ = actual_coordinate_cartesian.x;
            if (actual_coordinate_cartesian.y < min_y_)   min_y_ = actual_coordinate_cartesian.y;
            if (actual_coordinate_cartesian.y > max_y_)   max_y_ = actual_coordinate_cartesian.y;
        }
        for (int i=0; i<reference_targets_trajectories_UTM.size(); i++) {
            std::vector<geometry_msgs::Point32> actual_reference_target_trajectory_cartesian;
            for (int j=0; j<reference_targets_trajectories_UTM[i].size(); j++) {
                geometry_msgs::Point32 actual_coordinate_cartesian = UTM_to_cartesian (reference_targets_trajectories_UTM[i][j], reference_targets_trajectories_geo_[i][j], origin_coordinate_UTM, _origin_coordinates_geo);
                actual_reference_target_trajectory_cartesian.push_back(actual_coordinate_cartesian);
                if (actual_coordinate_cartesian.x < min_x_)   min_x_ = actual_coordinate_cartesian.x;
                if (actual_coordinate_cartesian.x > max_x_)   max_x_ = actual_coordinate_cartesian.x;
                if (actual_coordinate_cartesian.y < min_y_)   min_y_ = actual_coordinate_cartesian.y;
                if (actual_coordinate_cartesian.y > max_y_)   max_y_ = actual_coordinate_cartesian.y;
            }
            reference_targets_trajectories_cartesian_.push_back(actual_reference_target_trajectory_cartesian);
        }
        for (int i=0; i<geofencing_UTM.size(); i++) {
            geometry_msgs::Point32 actual_coordinate_cartesian = UTM_to_cartesian (geofencing_UTM[i], geofence_geo_[i], origin_coordinate_UTM, _origin_coordinates_geo);
            geofence_cartesian_.push_back(actual_coordinate_cartesian);
            if (actual_coordinate_cartesian.x < min_x_)   min_x_ = actual_coordinate_cartesian.x;
            if (actual_coordinate_cartesian.x > max_x_)   max_x_ = actual_coordinate_cartesian.x;
            if (actual_coordinate_cartesian.y < min_y_)   min_y_ = actual_coordinate_cartesian.y;
            if (actual_coordinate_cartesian.y > max_y_)   max_y_ = actual_coordinate_cartesian.y;
        }

        // Check if all the points are inside the geofence, if not KML not give error:
        for (int i=0; i<stations_cartesian_.size(); i++)                                                                                                  if ( ! checkIfPointInsidePolygon(geofence_cartesian_,stations_cartesian_[i]))                          { clearKMLparserClass(); /*std::cout << "KML Parser: there is at least one point outside of the geofence. Discarding KML." << std::endl;*/ return; }
        for (int i=0; i<no_fly_zones_cartesian_.size(); i++)                    for (int j=0; j<no_fly_zones_cartesian_[i].size(); j++)                   if ( ! checkIfPointInsidePolygon(geofence_cartesian_,no_fly_zones_cartesian_[i][j]))                   { clearKMLparserClass(); /*std::cout << "KML Parser: there is at least one point outside of the geofence. Discarding KML." << std::endl;*/ return; }
        for (int i=0; i<private_areas_cartesian_.size(); i++)                   for (int j=0; j<private_areas_cartesian_[i].size(); j++)                  if ( ! checkIfPointInsidePolygon(geofence_cartesian_,private_areas_cartesian_[i][j]))                  { clearKMLparserClass(); /*std::cout << "KML Parser: there is at least one point outside of the geofence. Discarding KML." << std::endl;*/ return; }
        for (int i=0; i<emergency_landing_sites_cartesian_.size(); i++)         for (int j=0; j<emergency_landing_sites_cartesian_[i].size(); j++)        if ( ! checkIfPointInsidePolygon(geofence_cartesian_,emergency_landing_sites_cartesian_[i][j]))        { clearKMLparserClass(); /*std::cout << "KML Parser: there is at least one point outside of the geofence. Discarding KML." << std::endl;*/ return; }
        for (int i=0; i<POIs_cartesian_.size(); i++)                                                                                                      if ( ! checkIfPointInsidePolygon(geofence_cartesian_,POIs_cartesian_[i]))                              { clearKMLparserClass(); /*std::cout << "KML Parser: there is at least one point outside of the geofence. Discarding KML." << std::endl;*/ return; }
        for (int i=0; i<roads_cartesian_.size(); i++)                           for (int j=0; j<roads_cartesian_[i].size(); j++)                          if ( ! checkIfPointInsidePolygon(geofence_cartesian_,roads_cartesian_[i][j]))                          { clearKMLparserClass(); /*std::cout << "KML Parser: there is at least one point outside of the geofence. Discarding KML." << std::endl;*/ return; }
        for (int i=0; i<rivers_or_lakes_cartesian_.size(); i++)                 for (int j=0; j<rivers_or_lakes_cartesian_[i].size(); j++)                if ( ! checkIfPointInsidePolygon(geofence_cartesian_,rivers_or_lakes_cartesian_[i][j]))                { clearKMLparserClass(); /*std::cout << "KML Parser: there is at least one point outside of the geofence. Discarding KML." << std::endl;*/ return; }
        for (int i=0; i<stadiums_cartesian_.size(); i++)                        for (int j=0; j<stadiums_cartesian_[i].size(); j++)                       if ( ! checkIfPointInsidePolygon(geofence_cartesian_,stadiums_cartesian_[i][j]))                       { clearKMLparserClass(); /*std::cout << "KML Parser: there is at least one point outside of the geofence. Discarding KML." << std::endl;*/ return; }
        for (int i=0; i<origin_of_formation_cartesian_.size(); i++)                                                                                       if ( ! checkIfPointInsidePolygon(geofence_cartesian_,origin_of_formation_cartesian_[i]))               { clearKMLparserClass(); /*std::cout << "KML Parser: there is at least one point outside of the geofence. Discarding KML." << std::endl;*/ return; }
        for (int i=0; i<reference_targets_trajectories_cartesian_.size(); i++)  for (int j=0; j<reference_targets_trajectories_cartesian_[i].size(); j++) if ( ! checkIfPointInsidePolygon(geofence_cartesian_,reference_targets_trajectories_cartesian_[i][j])) { clearKMLparserClass(); /*std::cout << "KML Parser: there is at least one point outside of the geofence. Discarding KML." << std::endl;*/ return; }

        if (min_x_==std::numeric_limits<double>::max() && max_x_==-std::numeric_limits<double>::max() && min_y_==std::numeric_limits<double>::max() && max_y_==-std::numeric_limits<double>::max()) {
            clearKMLparserClass();
            // std::cout << "KML Parser: something wrong happened parsing the KML. Invalid, broken or empty KML. Discarding KML. (1)" << std::endl;
            return;
        }

        parsed_correctly_ = true;

    } catch (...) { // catch any exception
        clearKMLparserClass();
        // std::cout << "KML Parser: something wrong happened parsing the KML. Invalid, broken or empty KML. Discarding KML. (2)" << std::endl;
        return;
    }

    // Check if there are the most important components in the KML: geoofence and stations.
    if (parsed_correctly_) {
        if (geofence_cartesian_.size()==0 && stations_cartesian_.size()==0) {
            clearKMLparserClass();
            // std::cout << "KML Parser: there are no geofencing and no stations in this KML. Discarding KML." << std::endl;
            return;
        } else if (geofence_cartesian_.size()==0) {
            clearKMLparserClass();
            // std::cout << "KML Parser: there is no geofencing in this KML. Discarding KML." << std::endl;
            return;
        }
        if (stations_cartesian_.size()==0) {
            clearKMLparserClass();
            // std::cout << "KML Parser: there are no stations in this KML. Discarding KML." << std::endl;
            return;
        }
    }

    // /////// DEBUG, show the results after converting UTM to cartesian with the first station as the origin (x=0, y=0, z=0). /////////
    // std::cout << "stations_cartesian_" << std::endl;                       for (int i=0; i<stations_cartesian_.size(); i++)                                                                                                  std::cout << std::setprecision (10) << stations_cartesian_[i] << std::endl;
    // std::cout << "no_fly_zones_cartesian_" << std::endl;                   for (int i=0; i<no_fly_zones_cartesian_.size(); i++)                    for (int j=0; j<no_fly_zones_cartesian_[i].size(); j++)                   std::cout << std::setprecision (10) << no_fly_zones_cartesian_[i][j] << std::endl;
    // std::cout << "private_areas_cartesian_" << std::endl;                  for (int i=0; i<private_areas_cartesian_.size(); i++)                   for (int j=0; j<private_areas_cartesian_[i].size(); j++)                  std::cout << std::setprecision (10) << private_areas_cartesian_[i][j] << std::endl;
    // std::cout << "emergency_landing_sites_cartesian_" << std::endl;        for (int i=0; i<emergency_landing_sites_cartesian_.size(); i++)         for (int j=0; j<emergency_landing_sites_cartesian_[i].size(); j++)        std::cout << std::setprecision (10) << emergency_landing_sites_cartesian_[i][j] << std::endl;
    // std::cout << "POIs_cartesian_" << std::endl;                           for (int i=0; i<POIs_cartesian_.size(); i++)                                                                                                      std::cout << std::setprecision (10) << POIs_cartesian_[i] << std::endl;
    // std::cout << "roads_cartesian_" << std::endl;                          for (int i=0; i<roads_cartesian_.size(); i++)                           for (int j=0; j<roads_cartesian_[i].size(); j++)                          std::cout << std::setprecision (10) << roads_cartesian_[i][j] << std::endl;
    // std::cout << "rivers_or_lakes_cartesian_" << std::endl;                for (int i=0; i<rivers_or_lakes_cartesian_.size(); i++)                 for (int j=0; j<rivers_or_lakes_cartesian_[i].size(); j++)                std::cout << std::setprecision (10) << rivers_or_lakes_cartesian_[i][j] << std::endl;
    // std::cout << "stadiums_cartesian_" << std::endl;                       for (int i=0; i<stadiums_cartesian_.size(); i++)                        for (int j=0; j<stadiums_cartesian_[i].size(); j++)                       std::cout << std::setprecision (10) << stadiums_cartesian_[i][j] << std::endl;
    // std::cout << "origin_of_formation_cartesian_" << std::endl;            for (int i=0; i<origin_of_formation_cartesian_.size(); i++)                                                                                       std::cout << std::setprecision (10) << origin_of_formation_cartesian_[i] << std::endl;
    // std::cout << "reference_targets_trajectories_cartesian_" << std::endl; for (int i=0; i<reference_targets_trajectories_cartesian_.size(); i++)  for (int j=0; j<reference_targets_trajectories_cartesian_[i].size(); j++) std::cout << std::setprecision (10) << reference_targets_trajectories_cartesian_[i][j] << std::endl;
    // std::cout << "geofence_cartesian_" << std::endl;                     for (int i=0; i<geofence_cartesian_.size(); i++)                                                                                                std::cout << std::setprecision (10) << geofence_cartesian_[i] << std::endl;
    // std::cout << "min_x_ = " << min_x_ << "   max_x_ = " << max_x_ << std::endl;
    // std::cout << "min_y_ = " << min_y_ << "   max_y_ = " << max_y_ << std::endl;
    // /////// DEBUG, show the results after converting UTM to cartesian with the first station as the origin (x=0, y=0, z=0). /////////

}   // end "runParser"



void KMLparser::clearKMLparserClass() {
    // Reset the attributes of KMLparser class
    stations_geo_.clear();
    no_fly_zones_geo_.clear();
    private_areas_geo_.clear();
    emergency_landing_sites_geo_.clear();
    POIs_geo_.clear();
    roads_geo_.clear();
    rivers_or_lakes_geo_.clear();
    stadiums_geo_.clear();
    origin_of_formation_geo_.clear();
    reference_targets_trajectories_geo_.clear();
    geofence_geo_.clear();

    stations_cartesian_.clear();
    no_fly_zones_cartesian_.clear();
    private_areas_cartesian_.clear();
    emergency_landing_sites_cartesian_.clear();
    POIs_cartesian_.clear();
    roads_cartesian_.clear();
    rivers_or_lakes_cartesian_.clear();
    stadiums_cartesian_.clear();
    origin_of_formation_cartesian_.clear();
    reference_targets_trajectories_cartesian_.clear();
    geofence_cartesian_.clear();

    min_x_ = 0;
    max_x_ = 0;
    min_y_ = 0;
    max_y_ = 0;

    parsed_correctly_ = false;
}   // end "clearKMLparserClass"



// Check if a test point is inside a polygon. Source: https://wrf.ecse.rpi.edu//Research/Short_Notes/pnpoly.html
bool KMLparser::checkIfPointInsidePolygon(const std::vector<geometry_msgs::Point32>& _polygon, const geometry_msgs::Point32& _test_point) {
    bool outside_or_inside = false;

    int i, j;

    // Toggle a boolean variable each time an infinite horizontal to the right (from the test point) crosses a vertex of the polygon.
    // If the number of crosses is even then the point is outside the polygon (variable is false), and if it's odd then it's inside (variable is true).
    for (i = 0, j = _polygon.size()-1; i < _polygon.size(); j = i++) {
        if ( ((_polygon[i].y>_test_point.y) != (_polygon[j].y>_test_point.y)) &&
        (_test_point.x < (_polygon[j].x-_polygon[i].x) * (_test_point.y-_polygon[i].y) / (_polygon[j].y-_polygon[i].y) + _polygon[i].x) ) {
            outside_or_inside = ! outside_or_inside;
        }
    }

    return outside_or_inside;   // Return false if _test_point is outside of _polygon, true if inside.
}   // end "checkIfPointInsidePolygon"



bool KMLparser::checkIfPointInsidePolygon(const geometry_msgs::Polygon& _polygon, const geometry_msgs::Point32& _test_point) {

    std::vector<geometry_msgs::Point32> polygon;
    for (int i=0; i<_polygon.points.size(); i++) polygon.push_back(_polygon.points[i]);

    return checkIfPointInsidePolygon(polygon, _test_point);

}   // end "checkIfPointInsidePolygon" ovetloaded for geometry_msgs::Polygon

} // end namespace multidrone
