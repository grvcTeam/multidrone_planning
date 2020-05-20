/**
 * MULTIDRONE Project:
 *
 * Multidrone KML parser. Made using the XML Parser "pugixml", version 1.8
 * 
 */

#ifndef KML_PARSER_H
#define KML_PARSER_H

#include <vector>
#include <string>
#include <geographic_msgs/GeoPoint.h>
#include <geometry_msgs/Point32.h>
#include <geometry_msgs/Polygon.h>

namespace multidrone {

/// KMLparser class that works as interface
class KMLparser {

public:
    KMLparser();                                // Default constructor, does nothing.
    KMLparser(const std::string& _KML_string, const geographic_msgs::GeoPoint& _origin_coordinates_geo);  // Overloaded constructor that receives the KML sting and call the parser method.
    ~KMLparser();

    void runParser(const std::string& _KML_string, const geographic_msgs::GeoPoint& _origin_coordinates_geo);   // Method that do the parse job.

    void clearKMLparserClass();     // Method to reset the attributes of the class.

    // The job of the KML parser is to fill the following needed attributes from the information contained in the KML:
    std::vector< geographic_msgs::GeoPoint >                 stations_geo_;                      // KML Placemark with <name>1</name> Vector of points that represent different stations.
    std::vector< std::vector<geographic_msgs::GeoPoint> >    no_fly_zones_geo_;                  // KML Placemark with <name>2</name> Each vector of points represent a polygon (no-fly zone) and there is a vector of no-fly zones.
    std::vector< std::vector<geographic_msgs::GeoPoint> >    private_areas_geo_;                 // KML Placemark with <name>3</name> Each vector of points represent a polygon (private area), and there is a vector of private areas.
    std::vector< std::vector<geographic_msgs::GeoPoint> >    emergency_landing_sites_geo_;       // KML Placemark with <name>4</name> Each vector of points represent a polygon (potential emergency landing site), and there is a vector of potential emergency landing sites.
    std::vector< geographic_msgs::GeoPoint >                 POIs_geo_;                          // KML Placemark with <name>5</name> Vector of points that represent different POIs.
    std::vector< std::vector<geographic_msgs::GeoPoint> >    roads_geo_;                         // KML Placemark with <name>6</name> Each vector of points represent a polygon (road), and there is a vector of roads.
    std::vector< std::vector<geographic_msgs::GeoPoint> >    rivers_or_lakes_geo_;               // KML Placemark with <name>7</name> Each vector of points represent a polygon (river or lake), and there is a vector of rivers and lakes.
    std::vector< std::vector<geographic_msgs::GeoPoint> >    stadiums_geo_;                      // KML Placemark with <name>8</name> Each vector of points represent a polygon (stadium), and there is a vector of stadiums.
    std::vector< geographic_msgs::GeoPoint >                 origin_of_formation_geo_;           // KML Placemark with <name>9</name> Vector of points that represent different Origin Of Formation Point.
    std::vector< std::vector<geographic_msgs::GeoPoint> >    reference_targets_trajectories_geo_;// KML Placemark with <name>10</name> Each vector of points represent a LineString (Reference Target Trajectory), and there is a vector of Reference Target Trajectories.
    std::vector<geographic_msgs::GeoPoint>                   geofence_geo_;                    // KML Placemark with <name>11</name> Only one vector of points that represents a polygon which is the geofence.
    // The points of the KML are all in geographic coordinates (longitude, latitude and altitude).

    // And also, the coordinates are transformed into Cartesian in "runParser":
    std::vector< geometry_msgs::Point32 >                 stations_cartesian_;
    std::vector< std::vector<geometry_msgs::Point32> >    no_fly_zones_cartesian_;
    std::vector< std::vector<geometry_msgs::Point32> >    private_areas_cartesian_;
    std::vector< std::vector<geometry_msgs::Point32> >    emergency_landing_sites_cartesian_;
    std::vector< geometry_msgs::Point32 >                 POIs_cartesian_;
    std::vector< std::vector<geometry_msgs::Point32> >    roads_cartesian_;
    std::vector< std::vector<geometry_msgs::Point32> >    rivers_or_lakes_cartesian_;
    std::vector< std::vector<geometry_msgs::Point32> >    stadiums_cartesian_;
    std::vector< geometry_msgs::Point32 >                 origin_of_formation_cartesian_;
    std::vector< std::vector<geometry_msgs::Point32> >    reference_targets_trajectories_cartesian_;
    std::vector<geometry_msgs::Point32>                   geofence_cartesian_;

    // Minimum and maximum values x and y in Cartesian coordinates (initialized to zero):
    double min_x_=0;
    double max_x_=0;
    double min_y_=0;
    double max_y_=0;

    bool parsed_correctly_ = false;

    static bool checkIfPointInsidePolygon(const std::vector<geometry_msgs::Point32>& _polygon, const geometry_msgs::Point32& _test_point);
    static bool checkIfPointInsidePolygon(const geometry_msgs::Polygon& _polygon, const geometry_msgs::Point32& _test_point);

};  // end KMLparser class

} // end namespace multidrone

#endif  // KML_PARSER_H