/**
 * MULTIDRONE Project:
 *
 * Path planner.
 * 
 */

#ifndef PATH_PLANNER_H
#define PATH_PLANNER_H

#include <ros/ros.h>
#include <string>
#include <vector>
#include <limits>
#include <utility>
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/Point32.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Polygon.h>
#include <geographic_msgs/GeoPoint.h>

#ifdef USING_IN_MULTIDRONE_PROJECT
#include <multidrone_kml_parser/multidrone_kml_parser.h>
#else
// Dummy KMLparser class when not using the specific KML parser of MultiDrone.
class KMLparser {
public:
    KMLparser() {}
    ~KMLparser() {}
    void runParser(const std::string& _KML_string, const geographic_msgs::GeoPoint& _origin_coordinates_geo) {}
    std::vector< std::vector<geometry_msgs::Point32> > no_fly_zones_cartesian_;
    std::vector< std::vector<geometry_msgs::Point32> > private_areas_cartesian_;
    std::vector<geometry_msgs::Point32>                geofence_cartesian_;
    // Minimum and maximum values x and y in Cartesian coordinates (initialized to zero):
    double min_x_=0;
    double max_x_=0;
    double min_y_=0;
    double max_y_=0;
};  // end KMLparser class
#endif

namespace multidrone {

/// PathPlanner class that works as interface
class PathPlanner {

public:
    PathPlanner();                       // Brief Constructor for the simplest case of path planning: return the final point, straight line.
    PathPlanner(const std::vector< std::vector<bool> >& _no_fly_zones, double _min_x, double _max_x, double _min_y, double _max_y);                 // Constructor that receives directly the no_fly_zones (rectangular boolean matrix) calculated out of the class. _min_x, _max_x, _min_y, _max_y are the x-y and coordinates of the limits of the grid. IMPORTANT: an empty (without obstacles) additional row or column will be added at each side of the given matrix, if this is a problem just add obstacles at the borders of the matrix map.
    PathPlanner(const std::string& _KML_string, const geographic_msgs::GeoPoint& _origin_coordinates_geo, unsigned int _max_grid_side = 100);  // Constructor that generates the grid of obstacles (no_fly_zones_) from the KML information. The input is the string that contains the KML and the size (in number of cells) of the bigger side of the rectanglular grid of obstacles, predefined at 100. IMPORTANT: the limits of the KML are expanded with an empty (without obstacles) row or column at each side of the KML no_fly_zone_ map, if this is a problem just add obstacles at the borders of the matrix map.
    PathPlanner(const std::vector<geometry_msgs::Polygon>& _obstacle_polygon_vector, std::vector<geometry_msgs::Point32> _geofence_cartesian, unsigned int _max_grid_side = 100);   // Constructor that does the same that the constructor with KML but receiving the obstacles list directly in a polygon vector, not inside a KML.

    ~PathPlanner();

    std::vector<geometry_msgs::PointStamped> getPath(const geometry_msgs::PointStamped& _initial_point_stamped, const geometry_msgs::PointStamped& _final_point_stamped, bool _movement_pattern=0, bool _show_results=0); // Method that returns a feasible path calculated with A* algorithm. All the waypoints at the height (z) of the final point (CAUTION).
    std::vector<geometry_msgs::PointStamped> getPathWithTimePredictions(const geometry_msgs::PointStamped& _initial_point_stamped, const geometry_msgs::PointStamped& _final_point_stamped, int _full_speed_xy, int _full_speed_z_down, int _full_speed_z_up, bool _movement_pattern=0, bool _show_results=0);   // "getPath" method that estimates the time in each waypoint given the speed of the drone: A* algorithm implementation that returns a path for one robot from his initial position to the end position.
    std::vector<geometry_msgs::PointStamped> getPathWithTimePredictionsAndInitialPoint(const geometry_msgs::PointStamped& _initial_point_stamped, const geometry_msgs::PointStamped& _final_point_stamped, int _full_speed_xy, int _full_speed_z_down, int _full_speed_z_up, bool _movement_pattern=0, bool _show_results=0); // The same but inserting the initial point in the trajectory.
    std::vector<geometry_msgs::Point32> getPath(const geometry_msgs::Point32& _initial_point, const geometry_msgs::Point32& _final_point, bool _movement_pattern=0, bool _show_results=0);                                // getPath but overloaded using Point32 instead of PointStamped.
    std::vector<geometry_msgs::Point> getPath(const geometry_msgs::Point& _initial_point, const geometry_msgs::Point& _final_point, bool _movement_pattern=0, bool _show_results=0);                                      // getPath but overloaded using Point instead of PointStamped.
    // The input of getPath are the initial and final points in cartesian coordinates. IMPORTANT: the origin of coordinates of these points and the origin of coordinates of the obstacles (no-fly zone matrix) must be the same.
    // Also, the method has two optional bool arguments:
    //      _movement_pattern:  if 0 (default) the movement of the agent can be in any direction, and if 1 the agent can only move in angles multiple of 45º (faster to compute if the grid is big, doesn't do visibility-loops).
    //      _show_results:      if 0 (default) no results are shown, and if 1 the map of obstacles and the path is plotted (usiing matplotlib-cpp), and also the path is shown in the terminal, with its cost and computation time.

    double getDistance();                           // Getter that returns path_distance_.
    double getFlatDistance();                       // Getter that returns path_flat_distance_.

    bool getTrivialPathPlannerOrNot();              // Return true if using trivial path planner, false if not.

    KMLparser KML_parser_from_path_planner_;

    static bool checkIfPointInsidePolygon(const std::vector<geometry_msgs::Point32>& _polygon, const geometry_msgs::Point32& _test_point);
    static bool checkIfPointInsidePolygon(const std::vector<geometry_msgs::Point32>& _polygon, const geometry_msgs::PointStamped& _test_point_stamped);
    static bool checkIfPointInsidePolygon(const geometry_msgs::Polygon& _polygon, const geometry_msgs::Point32& _test_point);

private:

    // Struct that will be initialized for each cell explored in the A* algorithm. The algorithm will construct a map of "CellInfo" (with a cell identifier as key) in order to reach the solution.
    struct CellInfo {
    public:
        unsigned int x;             // x coordinate of the cell in the grid (origin of x, x=0, is in the grid is on the left)
        unsigned int y;             // y coordinate of the cell in the grid (origin of y, y=0, is in the grid is on the top)
        unsigned int came_from;     // father of the cell (the one that minimize the cost if the cell is closed)
        unsigned int g_score = std::numeric_limits<unsigned int>::max();    // cost from the origin to the cell. Initialized at the beginning to "infinity" (maximum value possible for unsigned int) for each cell.
        unsigned int f_score = std::numeric_limits<unsigned int>::max();    // g_scores + heuristic cost from the cell to the goal position. Initialized at the beginning to "infinity" (maximum value possible for unsigned int) for each cell.
        bool closed = false;        // cell closed true if is fully studied
    };

    std::vector<std::vector<bool>> no_fly_zones_;   // Rectangular grid needed for the path planner algorithm. Elements with value of 0 (false) are part of the free space, and elements with value of 1 (true) contains obstacles.

    double x_cell_width_;    // Width in meters of each cell in the x axis.
    double y_cell_width_;    // Width in meters of each cell in the y axis.

    double path_distance_;                          // Sum of norms between waypoints, initialized in getPath.
    double path_flat_distance_;                     // Sum of norms between waypoints supposed flat path, initialized in getPath.

    // TODO: update the no_fly_zones_ with dynamic changes in the enviroment.

    std::vector< std::pair<double,double> > calculateIntersectionsOfSegmentWithGrid (double first_absolute_point_of_segment_x, double first_absolute_point_of_segment_y, double last_absolute_point_of_segment_x, double last_absolute_point_of_segment_y);
    void fillCellsWithObstacles (const std::vector< std::pair<double,double> >& points_intersections_of_segment_with_grid);
    bool visibilityCheck (const std::vector< std::pair<double,double> >& points_intersections_of_segment_with_grid);

    bool trivial_path_planner_ = false;

};  // end PathPlanner class

}   // end namespace multidrone

#endif  // PATH_PLANNER_H