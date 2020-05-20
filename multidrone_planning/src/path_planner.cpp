/**
 * MULTIDRONE Project:
 *
 * Path planner.
 * 
 */

#include <multidrone_planning/path_planner.h>

#include <map>
#include <math.h>
#include <cmath>
#include <iterator>
#include <iostream>
#include <fstream>
#include <sstream>
#include <typeinfo>
#include <time.h>
#include <algorithm>

// #define DEBUG_MODE       // UNCOMMENT FOR VISUALIZATION OF RESULTS

#ifdef DEBUG_MODE
#include "matplotlibcpp/matplotlibcpp.h"    // matplotlib-cpp has a MIT License (MIT), Copyright (c) 2014 Benno Evers. The full license description of matplotlib, matplotlib-cpp and its README can be found at its root.
namespace plt = matplotlibcpp;              // namespace-alias-definition: makes a synonym of another namespace: see namespace alias
#endif

namespace multidrone {

// Brief Constructor for the simplest case of path planning: return the final point, straight line.
PathPlanner::PathPlanner() {
    trivial_path_planner_ = true;
    x_cell_width_ = -1;
    y_cell_width_ = -1;
}   // end constructor "PathPlanner" for trivial case



// Brief Constructor that receives directly the no_fly_zones (rectangular boolean matrix) calculated out of the class. This obstacle map will be used by the A* algorithm.
// IMPORTANT: an empty (without obstacles) additional row or column will be added at each side of the given matrix. If this is a problem just add obstacles at the borders of the matrix map.
PathPlanner::PathPlanner(const std::vector< std::vector<bool> >& _no_fly_zones, double _min_x, double _max_x, double _min_y, double _max_y) { // _no_fly_zones is the received grid obstacle matrix. _min_x, _max_x, _min_y, _max_y are the x-y and coordinates of the limits of the grid.
    trivial_path_planner_ = false;

    KML_parser_from_path_planner_.geofence_cartesian_.clear();
    geometry_msgs::Point32 empty_point;
    KML_parser_from_path_planner_.geofence_cartesian_.push_back(empty_point);

    bool initial_value = 0;
    unsigned int y_side_size_cells_in_grid = _no_fly_zones.size();         // Width in cells of the grid.
    unsigned int x_side_size_cells_in_grid = _no_fly_zones[0].size();      // Width in cells of the grid.
    no_fly_zones_.resize(y_side_size_cells_in_grid, std::vector<bool>(x_side_size_cells_in_grid, initial_value));
    no_fly_zones_ = _no_fly_zones;                  // Once resized the class atribute, copy the given grid into the attribute.

    double x_side_size_meters = _max_x - _min_x;    // Width in terms of number of grid cells for the map's x axis.
    double y_side_size_meters = _max_y - _min_y;    // Width in terms of number of grid cells for the map's y axis.

    x_cell_width_ = x_side_size_meters/x_side_size_cells_in_grid;   // Width in meters of each cell in the x axis.
    y_cell_width_ = y_side_size_meters/y_side_size_cells_in_grid;   // Width in meters of each cell in the y axis.

    // Insert empty column at the beginning and end of the matrix:
    for (int i=0; i<no_fly_zones_.size(); i++) {
        no_fly_zones_[i].insert(no_fly_zones_[i].begin(), 0);
        no_fly_zones_[i].push_back(0);
    }

    // Insert empty row at the beginning and end of the matrix:
    std::vector<bool> empty_row; empty_row.resize(no_fly_zones_[0].size(), initial_value);
    no_fly_zones_.insert(no_fly_zones_.begin(), empty_row);
    no_fly_zones_.push_back(empty_row);

    // Charge in the KMLparser class the min and max values updated with the new empty columns and rows at each side of the matrix:
    KML_parser_from_path_planner_.min_x_ = _min_x;
    KML_parser_from_path_planner_.max_x_ = _max_x;
    KML_parser_from_path_planner_.min_y_ = _min_y;
    KML_parser_from_path_planner_.max_y_ = _max_y;

#ifdef DEBUG_MODE
    // Show the map in the terminal.
    for (int i=no_fly_zones_.size()-1;i>=0;i--)         // IMPORTANT: visual matrix represented upside-down!! Thats because the y axis is backwards when treated as matrix.
    {
        for (int j=0;j<no_fly_zones_[0].size();j++)
        {
            std::cout << no_fly_zones_[i][j] << ' ';
        }
        std::cout << std::endl;
    }
#endif

}   // end constructor "PathPlanner" given the _no_fly_zones matrix.



// Constructor that generates the obstacle map (no-fly zones) for the A* algorithm from the KML string.
// IMPORTANT: the limits of the KML are expanded with an empty (without obstacles) row or column at each side of the KML no_fly_zone_ map, if this is a problem just add obstacles at the borders of the matrix map.
PathPlanner::PathPlanner(const std::string& _KML_string, const geographic_msgs::GeoPoint& _origin_coordinates_geo, unsigned int _max_grid_side) { // _max_grid_side is the number of cells that the grid has in the bigger side of the rectangular map of obstacles (predefined in the header file). KML_string is the string that contains the KML.
    trivial_path_planner_ = false;

    path_distance_ = std::numeric_limits<double>::max();      // Distance initialized to "infinity" (maximum value possible for double).
    path_flat_distance_ = std::numeric_limits<double>::max(); // Distance initialized to "infinity" (maximum value possible for double).

    KML_parser_from_path_planner_.runParser( _KML_string, _origin_coordinates_geo );

    if (KML_parser_from_path_planner_.parsed_correctly_) {

        double x_side_size_meters = KML_parser_from_path_planner_.max_x_ - KML_parser_from_path_planner_.min_x_;    // Width in meters of the map's x axis.
        double y_side_size_meters = KML_parser_from_path_planner_.max_y_ - KML_parser_from_path_planner_.min_y_;    // Width in meters of the map's y axis.
        unsigned int x_side_size_cells_in_grid = x_side_size_meters >= y_side_size_meters ? (_max_grid_side-2) : (unsigned int) (_max_grid_side-2) * x_side_size_meters/y_side_size_meters;  // Width in terms of number of grid cells for the map's x axis.
        unsigned int y_side_size_cells_in_grid = y_side_size_meters >= x_side_size_meters ? (_max_grid_side-2) : (unsigned int) (_max_grid_side-2) * y_side_size_meters/x_side_size_meters;  // Width in terms of number of grid cells for the map's y axis.

        x_cell_width_ = x_side_size_meters/x_side_size_cells_in_grid;   // Width in meters of each cell in the x axis.
        y_cell_width_ = y_side_size_meters/y_side_size_cells_in_grid;   // Width in meters of each cell in the y axis.

        x_side_size_meters += 2*x_cell_width_; // The map will be 1 cell bigger at each side of the rectangle.
        y_side_size_meters += 2*y_cell_width_; // The map will be 1 cell bigger at each side of the rectangle.

        x_side_size_cells_in_grid += 2;    // Number of columns of the array. +2 because the grid will have at least one free fly zone at every side of the map.
        y_side_size_cells_in_grid += 2;    // Number of rows of the array. +2 because the grid will have at least one free fly zone at every side of the map.

        // Reset to zero the map with the necessary size, no obstacles yet.
        bool initial_value = 0;
        no_fly_zones_.resize(y_side_size_cells_in_grid, std::vector<bool>(x_side_size_cells_in_grid, initial_value));

        // Now is time to introduce the obstacles in the map with the information from the parser. The obstacles will be "no-fly zones" and "privare areas".
        // It's important to understand that the points of the parser (which can be positive or negative) are transformed to points in the grid, where (x,y) >=0. The grid will be rectangular, having a configurable maximum number of cells in the greater side.

        for (int i=0; i<KML_parser_from_path_planner_.no_fly_zones_cartesian_.size(); i++) {          // for every no-fly zone polygon...
            for (int j=0; j<KML_parser_from_path_planner_.no_fly_zones_cartesian_[i].size()-1; j++) { // The pairs of points [j] to [j+1] from the polygon are checked forming a segment. The intersection of that segment with the lines of the grid will indicate the cells with obstacles.
                std::vector< std::pair<double,double> > points_intersections_of_segment_with_grid = calculateIntersectionsOfSegmentWithGrid (KML_parser_from_path_planner_.no_fly_zones_cartesian_[i][j].x - KML_parser_from_path_planner_.min_x_ + x_cell_width_ , KML_parser_from_path_planner_.no_fly_zones_cartesian_[i][j].y - KML_parser_from_path_planner_.min_y_ + y_cell_width_ , KML_parser_from_path_planner_.no_fly_zones_cartesian_[i][j+1].x - KML_parser_from_path_planner_.min_x_ + x_cell_width_ , KML_parser_from_path_planner_.no_fly_zones_cartesian_[i][j+1].y - KML_parser_from_path_planner_.min_y_ + y_cell_width_);

                fillCellsWithObstacles (points_intersections_of_segment_with_grid);

#ifdef DEBUG_MODE
                // Plot points_intersections_of_segment_with_grid for no_fly_zones_. 
                std::vector<double> x_plot, y_plot;
                for (int k=0; k<points_intersections_of_segment_with_grid.size(); k++) { x_plot.push_back(points_intersections_of_segment_with_grid[k].first); y_plot.push_back(points_intersections_of_segment_with_grid[k].second); }
                plt::plot(x_plot,y_plot,"xg-");
#endif
            }
        }

        for (int i=0; i<KML_parser_from_path_planner_.private_areas_cartesian_.size(); i++) {          // for every no-fly zone polygon...
            for (int j=0; j<KML_parser_from_path_planner_.private_areas_cartesian_[i].size()-1; j++) { // The pairs of points [j] to [j+1] from the polygon are checked forming a segment. The intersection of that segment with the lines of the grid will indicate the cells with obstacles.
                std::vector< std::pair<double,double> > points_intersections_of_segment_with_grid = calculateIntersectionsOfSegmentWithGrid (KML_parser_from_path_planner_.private_areas_cartesian_[i][j].x - KML_parser_from_path_planner_.min_x_ + x_cell_width_ , KML_parser_from_path_planner_.private_areas_cartesian_[i][j].y - KML_parser_from_path_planner_.min_y_ + y_cell_width_ , KML_parser_from_path_planner_.private_areas_cartesian_[i][j+1].x - KML_parser_from_path_planner_.min_x_ + x_cell_width_ , KML_parser_from_path_planner_.private_areas_cartesian_[i][j+1].y - KML_parser_from_path_planner_.min_y_ + y_cell_width_);

                fillCellsWithObstacles (points_intersections_of_segment_with_grid);

#ifdef DEBUG_MODE
                // Plot points_intersections_of_segment_with_grid for private_areas_.
                std::vector<double> x_plot, y_plot;
                for (int k=0; k<points_intersections_of_segment_with_grid.size(); k++) { x_plot.push_back(points_intersections_of_segment_with_grid[k].first); y_plot.push_back(points_intersections_of_segment_with_grid[k].second); }
                plt::plot(x_plot,y_plot,"xg-");
#endif
            }
        }

        for (int j=0; j<KML_parser_from_path_planner_.geofence_cartesian_.size()-1; j++) { // The pairs of points [j] to [j+1] from the polygon are checked forming a segment. The intersection of that segment with the lines of the grid will indicate the cells with obstacles.
            std::vector< std::pair<double,double> > points_intersections_of_segment_with_grid = calculateIntersectionsOfSegmentWithGrid (KML_parser_from_path_planner_.geofence_cartesian_[j].x - KML_parser_from_path_planner_.min_x_ + x_cell_width_ , KML_parser_from_path_planner_.geofence_cartesian_[j].y - KML_parser_from_path_planner_.min_y_ + y_cell_width_ , KML_parser_from_path_planner_.geofence_cartesian_[j+1].x - KML_parser_from_path_planner_.min_x_ + x_cell_width_ , KML_parser_from_path_planner_.geofence_cartesian_[j+1].y - KML_parser_from_path_planner_.min_y_ + y_cell_width_);

            fillCellsWithObstacles (points_intersections_of_segment_with_grid);

#ifdef DEBUG_MODE
            // Plot points_intersections_of_segment_with_grid for private_areas_.
            std::vector<double> x_plot, y_plot;
            for (int k=0; k<points_intersections_of_segment_with_grid.size(); k++) { x_plot.push_back(points_intersections_of_segment_with_grid[k].first); y_plot.push_back(points_intersections_of_segment_with_grid[k].second); }
            plt::plot(x_plot,y_plot,"xg-");
#endif
        }

#ifdef DEBUG_MODE
        /////////////////// Show the map in the terminal. ///////////////////
        for (int i=no_fly_zones_.size()-1;i>=0;i--)         // IMPORTANT: visual matrix represented upside-down!! Thats because the y axis is backwards when treated as matrix.
        {
            for (int j=0;j<no_fly_zones_[0].size();j++)
            {
                std::cout << no_fly_zones_[i][j] << ' ';
            }
            std::cout << std::endl;
        }
        /////////////////// Show the map in the terminal. ///////////////////

        /////////////////////// Plot grid and obstacles (not the path). ///////////////////////
        // plot vertical lines of the grid:
        for (int k=0; k<=x_side_size_cells_in_grid;k++) {
            std::vector<double> x_plot, y_plot;
            x_plot.push_back(k*x_cell_width_); x_plot.push_back(k*x_cell_width_);
            y_plot.push_back(0); y_plot.push_back(y_side_size_meters);
            plt::plot(x_plot,y_plot,"b");
        }

        // plot horizontal lines of the grid:
        for (int k=0; k<=y_side_size_cells_in_grid;k++) {
            std::vector<double> x_plot, y_plot;
            x_plot.push_back(0); x_plot.push_back(x_side_size_meters);
            y_plot.push_back(k*y_cell_width_); y_plot.push_back(k*y_cell_width_);
            plt::plot(x_plot,y_plot,"b");
        }

        // fill the inside of the cells with obstacles with a cross:
        for (int i=0;i<no_fly_zones_.size();i++)
        {
            for (int j=0;j<no_fly_zones_[0].size();j++)
            {
                if (no_fly_zones_[i][j]==1) {
                    double x_1=j*x_cell_width_;     double x_2=(j+1)*x_cell_width_;
                    double y_1=i*y_cell_width_;     double y_2=(i+1)*y_cell_width_;
                    std::vector<double> x_plot, y_plot;
                    x_plot.push_back(x_1);          x_plot.push_back(x_2);
                    y_plot.push_back(y_1);          y_plot.push_back(y_2);
                    plt::plot(x_plot, y_plot, "r");
                    x_plot.clear(), y_plot.clear();
                    x_plot.push_back(x_1);          x_plot.push_back(x_2);
                    y_plot.push_back(y_2);          y_plot.push_back(y_1);
                    plt::plot(x_plot, y_plot, "r");
                }
            }
        }

        // Draw (show) everything:
        plt::show();
        /////////////////////// Plot grid and obstacles (not the path). ///////////////////////
#endif
    } else {
        // If parsed with errors (KML_parser_from_path_planner_.parsed_correctly_ == false), return trivial Path Planner:
        trivial_path_planner_ = true;
        x_cell_width_ = -1;
        y_cell_width_ = -1;
    }

}   // end constructor "PathPlanner" with KML string.



// Constructor that generates the obstacle map (no-fly zones) for the A* algorithm from the a vector of polygons.
// IMPORTANT: the limits of the obstacles are expanded with an empty (without obstacles) row or column at each side of the KML no_fly_zone_ map, if this is a problem just add obstacles at the borders of the matrix map.
PathPlanner::PathPlanner(const std::vector<geometry_msgs::Polygon>& _obstacle_polygon_vector, std::vector<geometry_msgs::Point32> _geofence_cartesian, unsigned int _max_grid_side) { // _max_grid_side is the number of cells that the grid has in the bigger side of the rectangular map of obstacles (predefined in the header file). KML_string is the string that contains the KML.
    trivial_path_planner_ = false;

    path_distance_ = std::numeric_limits<double>::max();      // Distance initialized to "infinity" (maximum value possible for double).
    path_flat_distance_ = std::numeric_limits<double>::max(); // Distance initialized to "infinity" (maximum value possible for double).

    KML_parser_from_path_planner_.geofence_cartesian_.clear();
    KML_parser_from_path_planner_.geofence_cartesian_ = _geofence_cartesian;

    KML_parser_from_path_planner_.no_fly_zones_cartesian_.clear();

    KML_parser_from_path_planner_.min_x_ =  std::numeric_limits<double>::max() ;   // Minimum coordinate initialized to "infinity" (maximum value possible for double).
    KML_parser_from_path_planner_.max_x_ = -std::numeric_limits<double>::max() ;   // Maximum coordinate initialized to minus "infinity" (minimum value possible for double).
    KML_parser_from_path_planner_.min_y_ =  std::numeric_limits<double>::max() ;   // Minimum coordinate initialized to "infinity" (maximum value possible for double).
    KML_parser_from_path_planner_.max_y_ = -std::numeric_limits<double>::max() ;   // Maximum coordinate initialized to minus "infinity" (minimum value possible for double).

    for (int i=0; i<_obstacle_polygon_vector.size(); i++) {
        KML_parser_from_path_planner_.no_fly_zones_cartesian_.push_back(_obstacle_polygon_vector[i].points);

        for (int j=0; j<_obstacle_polygon_vector[i].points.size(); j++) {
            if (_obstacle_polygon_vector[i].points[j].x < KML_parser_from_path_planner_.min_x_)  KML_parser_from_path_planner_.min_x_ = _obstacle_polygon_vector[i].points[j].x;
            if (_obstacle_polygon_vector[i].points[j].x > KML_parser_from_path_planner_.max_x_)  KML_parser_from_path_planner_.max_x_ = _obstacle_polygon_vector[i].points[j].x;
            if (_obstacle_polygon_vector[i].points[j].y < KML_parser_from_path_planner_.min_y_)  KML_parser_from_path_planner_.min_y_ = _obstacle_polygon_vector[i].points[j].y;
            if (_obstacle_polygon_vector[i].points[j].y > KML_parser_from_path_planner_.max_y_)  KML_parser_from_path_planner_.max_y_ = _obstacle_polygon_vector[i].points[j].y;
        }
    }

    double x_side_size_meters = KML_parser_from_path_planner_.max_x_ - KML_parser_from_path_planner_.min_x_;    // Width in meters of the map's x axis.
    double y_side_size_meters = KML_parser_from_path_planner_.max_y_ - KML_parser_from_path_planner_.min_y_;    // Width in meters of the map's y axis.
    unsigned int x_side_size_cells_in_grid = x_side_size_meters >= y_side_size_meters ? (_max_grid_side-2) : (unsigned int) (_max_grid_side-2) * x_side_size_meters/y_side_size_meters;  // Width in terms of number of grid cells for the map's x axis.
    unsigned int y_side_size_cells_in_grid = y_side_size_meters >= x_side_size_meters ? (_max_grid_side-2) : (unsigned int) (_max_grid_side-2) * y_side_size_meters/x_side_size_meters;  // Width in terms of number of grid cells for the map's y axis.

    x_cell_width_ = x_side_size_meters/x_side_size_cells_in_grid;   // Width in meters of each cell in the x axis.
    y_cell_width_ = y_side_size_meters/y_side_size_cells_in_grid;   // Width in meters of each cell in the y axis.

    x_side_size_meters += 2*x_cell_width_; // The map will be 1 cell bigger at each side of the rectangle.
    y_side_size_meters += 2*y_cell_width_; // The map will be 1 cell bigger at each side of the rectangle.

    x_side_size_cells_in_grid += 2;    // Number of columns of the array. +2 because the grid will have at least one free fly zone at every side of the map.
    y_side_size_cells_in_grid += 2;    // Number of rows of the array. +2 because the grid will have at least one free fly zone at every side of the map.

    // Reset to zero the map with the necessary size, no obstacles yet.
    bool initial_value = 0;
    no_fly_zones_.resize(y_side_size_cells_in_grid, std::vector<bool>(x_side_size_cells_in_grid, initial_value));

    // Now is time to introduce the obstacles in the map with the information from the parser. The obstacles will be "no-fly zones" and "privare areas".
    // It's important to understand that the points of the parser (which can be positive or negative) are transformed to points in the grid, where (x,y) >=0. The grid will be rectangular, having a configurable maximum number of cells in the greater side.

    for (int i=0; i<KML_parser_from_path_planner_.no_fly_zones_cartesian_.size(); i++) {          // for every no-fly zone polygon...
        for (int j=0; j<KML_parser_from_path_planner_.no_fly_zones_cartesian_[i].size()-1; j++) { // The pairs of points [j] to [j+1] from the polygon are checked forming a segment. The intersection of that segment with the lines of the grid will indicate the cells with obstacles.
            std::vector< std::pair<double,double> > points_intersections_of_segment_with_grid = calculateIntersectionsOfSegmentWithGrid (KML_parser_from_path_planner_.no_fly_zones_cartesian_[i][j].x - KML_parser_from_path_planner_.min_x_ + x_cell_width_ , KML_parser_from_path_planner_.no_fly_zones_cartesian_[i][j].y - KML_parser_from_path_planner_.min_y_ + y_cell_width_ , KML_parser_from_path_planner_.no_fly_zones_cartesian_[i][j+1].x - KML_parser_from_path_planner_.min_x_ + x_cell_width_ , KML_parser_from_path_planner_.no_fly_zones_cartesian_[i][j+1].y - KML_parser_from_path_planner_.min_y_ + y_cell_width_);

            fillCellsWithObstacles (points_intersections_of_segment_with_grid);

#ifdef DEBUG_MODE
            // Plot points_intersections_of_segment_with_grid for no_fly_zones_. 
            std::vector<double> x_plot, y_plot;
            for (int k=0; k<points_intersections_of_segment_with_grid.size(); k++) { x_plot.push_back(points_intersections_of_segment_with_grid[k].first); y_plot.push_back(points_intersections_of_segment_with_grid[k].second); }
            plt::plot(x_plot,y_plot,"xg-");
#endif
        }
    }

    for (int j=0; j<KML_parser_from_path_planner_.geofence_cartesian_.size()-1; j++) { // The pairs of points [j] to [j+1] from the polygon are checked forming a segment. The intersection of that segment with the lines of the grid will indicate the cells with obstacles.
        std::vector< std::pair<double,double> > points_intersections_of_segment_with_grid = calculateIntersectionsOfSegmentWithGrid (KML_parser_from_path_planner_.geofence_cartesian_[j].x - KML_parser_from_path_planner_.min_x_ + x_cell_width_ , KML_parser_from_path_planner_.geofence_cartesian_[j].y - KML_parser_from_path_planner_.min_y_ + y_cell_width_ , KML_parser_from_path_planner_.geofence_cartesian_[j+1].x - KML_parser_from_path_planner_.min_x_ + x_cell_width_ , KML_parser_from_path_planner_.geofence_cartesian_[j+1].y - KML_parser_from_path_planner_.min_y_ + y_cell_width_);

        fillCellsWithObstacles (points_intersections_of_segment_with_grid);

#ifdef DEBUG_MODE
        // Plot points_intersections_of_segment_with_grid for private_areas_.
        std::vector<double> x_plot, y_plot;
        for (int k=0; k<points_intersections_of_segment_with_grid.size(); k++) { x_plot.push_back(points_intersections_of_segment_with_grid[k].first); y_plot.push_back(points_intersections_of_segment_with_grid[k].second); }
        plt::plot(x_plot,y_plot,"xg-");
#endif
    }

#ifdef DEBUG_MODE
    /////////////////// Show the map in the terminal. ///////////////////
    for (int i=no_fly_zones_.size()-1;i>=0;i--)         // IMPORTANT: visual matrix represented upside-down!! Thats because the y axis is backwards when treated as matrix.
    {
        for (int j=0;j<no_fly_zones_[0].size();j++)
        {
            std::cout << no_fly_zones_[i][j] << ' ';
        }
        std::cout << std::endl;
    }
    /////////////////// Show the map in the terminal. ///////////////////

    /////////////////////// Plot grid and obstacles (not the path). ///////////////////////
    // plot vertical lines of the grid:
    for (int k=0; k<=x_side_size_cells_in_grid;k++) {
        std::vector<double> x_plot, y_plot;
        x_plot.push_back(k*x_cell_width_); x_plot.push_back(k*x_cell_width_);
        y_plot.push_back(0); y_plot.push_back(y_side_size_meters);
        plt::plot(x_plot,y_plot,"b");
    }

    // plot horizontal lines of the grid:
    for (int k=0; k<=y_side_size_cells_in_grid;k++) {
        std::vector<double> x_plot, y_plot;
        x_plot.push_back(0); x_plot.push_back(x_side_size_meters);
        y_plot.push_back(k*y_cell_width_); y_plot.push_back(k*y_cell_width_);
        plt::plot(x_plot,y_plot,"b");
    }

    // fill the inside of the cells with obstacles with a cross:
    for (int i=0;i<no_fly_zones_.size();i++)
    {
        for (int j=0;j<no_fly_zones_[0].size();j++)
        {
            if (no_fly_zones_[i][j]==1) {
                double x_1=j*x_cell_width_;     double x_2=(j+1)*x_cell_width_;
                double y_1=i*y_cell_width_;     double y_2=(i+1)*y_cell_width_;
                std::vector<double> x_plot, y_plot;
                x_plot.push_back(x_1);          x_plot.push_back(x_2);
                y_plot.push_back(y_1);          y_plot.push_back(y_2);
                plt::plot(x_plot, y_plot, "r");
                x_plot.clear(), y_plot.clear();
                x_plot.push_back(x_1);          x_plot.push_back(x_2);
                y_plot.push_back(y_2);          y_plot.push_back(y_1);
                plt::plot(x_plot, y_plot, "r");
            }
        }
    }

    // Draw (show) everything:
    plt::show();
    /////////////////////// Plot grid and obstacles (not the path). ///////////////////////
#endif

}   // end constructor "PathPlanner" with polygon's vector of obstacles.



// Brief Destructor
PathPlanner::~PathPlanner() {}



// "getPath" method: A* algorithm implementation that returns a path for one robot from his initial position to the end position. All the waypoints returned at the height (z) of the final point (CAUTION).
std::vector<geometry_msgs::PointStamped> PathPlanner::getPath(const geometry_msgs::PointStamped& _initial_point_stamped, const geometry_msgs::PointStamped& _final_point_stamped, bool _movement_pattern, bool _show_results) {

    // The input of getPath are the initial and final points in cartesian coordinates. IMPORTANT: the origin of coordinates of these points and the origin of coordinates of the obstacles (no-fly zone matrix) must be the same.
    // Also, the function has two optional bool arguments:
    //      _movement_pattern:  if 0 (default) the movement of the agent can be in any direction, and if 1 the agent can only move in angles multiple of 45º (faster to compute if the grid is big, doesn't do visibility-loops).
    //      _show_results:      if 0 (default) no results are shown, and if 1 the map of obstacles and the path is plotted (usiing matplotlib-cpp), and also the path is shown in the terminal, with its cost and computation time.

    clock_t t_begin;
    clock_t t_end;
    if ( _show_results ) {
        ////////////////// Testing, calculate time //////////////////
        t_begin = clock();
        ////////////////// Testing, calculate time //////////////////
    }

    std::vector<geometry_msgs::PointStamped> path_cells;             // The path expressed in cells.
    std::vector<geometry_msgs::PointStamped> path;                   // The output of this method.

    path_distance_ = std::numeric_limits<double>::max(); // Distance initialized to "infinity" (maximum value possible for double).
    path_flat_distance_ = std::numeric_limits<double>::max(); // Distance initialized to "infinity" (maximum value possible for double).

    if ( x_cell_width_==-1 && y_cell_width_==-1 ) {  // Simplest case of path planning (constructor without inputs): return the final point, straight line.
        path.push_back( _final_point_stamped );
        path_distance_ = sqrt( pow(_final_point_stamped.point.x-_initial_point_stamped.point.x,2) + pow(_final_point_stamped.point.y-_initial_point_stamped.point.y,2) + pow(_final_point_stamped.point.z-_initial_point_stamped.point.z,2) );
        path_flat_distance_ = sqrt( pow(_final_point_stamped.point.x-_initial_point_stamped.point.x,2) + pow(_final_point_stamped.point.y-_initial_point_stamped.point.y,2) );
        return path;
    }

    if ( KML_parser_from_path_planner_.geofence_cartesian_.size()<=1 ) {

        if ( KML_parser_from_path_planner_.geofence_cartesian_.size()==0 ) ROS_WARN("Path Planner: warning, empty geofence.");

        // If the initial and/or final points are outside of the KML map, finish the algorithm and return an empty path. 
        if ( (_initial_point_stamped.point.x<KML_parser_from_path_planner_.min_x_-x_cell_width_) || (_initial_point_stamped.point.y<KML_parser_from_path_planner_.min_y_-y_cell_width_) || (_initial_point_stamped.point.x>KML_parser_from_path_planner_.max_x_+x_cell_width_) || (_initial_point_stamped.point.y>KML_parser_from_path_planner_.max_y_+y_cell_width_) || (_final_point_stamped.point.x<KML_parser_from_path_planner_.min_x_-x_cell_width_) || (_final_point_stamped.point.y<KML_parser_from_path_planner_.min_y_-y_cell_width_) || (_final_point_stamped.point.x>KML_parser_from_path_planner_.max_x_+x_cell_width_) || (_final_point_stamped.point.y>KML_parser_from_path_planner_.max_y_+y_cell_width_) ) {
            if ( ( (_initial_point_stamped.point.x<KML_parser_from_path_planner_.min_x_-x_cell_width_) || (_initial_point_stamped.point.y<KML_parser_from_path_planner_.min_y_-y_cell_width_) || (_initial_point_stamped.point.x>KML_parser_from_path_planner_.max_x_+x_cell_width_) || (_initial_point_stamped.point.y>KML_parser_from_path_planner_.max_y_+y_cell_width_) ) && ( (_final_point_stamped.point.x<KML_parser_from_path_planner_.min_x_-x_cell_width_) || (_final_point_stamped.point.y<KML_parser_from_path_planner_.min_y_-y_cell_width_) || (_final_point_stamped.point.x>KML_parser_from_path_planner_.max_x_+x_cell_width_) || (_final_point_stamped.point.y>KML_parser_from_path_planner_.max_y_+y_cell_width_) ) )
                ROS_ERROR("Path Planner: initial and final points out of the KML map. Returning empty path.");
            else if ( (_initial_point_stamped.point.x<KML_parser_from_path_planner_.min_x_-x_cell_width_) || (_initial_point_stamped.point.y<KML_parser_from_path_planner_.min_y_-y_cell_width_) || (_initial_point_stamped.point.x>KML_parser_from_path_planner_.max_x_+x_cell_width_) || (_initial_point_stamped.point.y>KML_parser_from_path_planner_.max_y_+y_cell_width_) )
                ROS_ERROR("Path Planner: initial point out of the KML map. Returning empty path.");
            else if ( (_final_point_stamped.point.x<KML_parser_from_path_planner_.min_x_-x_cell_width_) || (_final_point_stamped.point.y<KML_parser_from_path_planner_.min_y_-y_cell_width_) || (_final_point_stamped.point.x>KML_parser_from_path_planner_.max_x_+x_cell_width_) || (_final_point_stamped.point.y>KML_parser_from_path_planner_.max_y_+y_cell_width_) )
                ROS_ERROR("Path Planner: final point out of the KML map. Returning empty path.");
            return path;
        }
    } else {
        // If the initial and/or final points are outside of the geofence, finish the algorithm and return an empty path. 
        if ( !checkIfPointInsidePolygon(KML_parser_from_path_planner_.geofence_cartesian_, _initial_point_stamped) || !checkIfPointInsidePolygon(KML_parser_from_path_planner_.geofence_cartesian_, _final_point_stamped) ) {
            if ( !checkIfPointInsidePolygon(KML_parser_from_path_planner_.geofence_cartesian_, _initial_point_stamped) && !checkIfPointInsidePolygon(KML_parser_from_path_planner_.geofence_cartesian_, _final_point_stamped) )
                ROS_ERROR("Path Planner: initial and final points out of the geofencing. Returning empty path.");
            else if ( !checkIfPointInsidePolygon(KML_parser_from_path_planner_.geofence_cartesian_, _initial_point_stamped) )
                ROS_ERROR("Path Planner: initial point out of the geofencing. Returning empty path.");
            else if ( !checkIfPointInsidePolygon(KML_parser_from_path_planner_.geofence_cartesian_, _final_point_stamped) )
                ROS_ERROR("Path Planner: final point out of the geofencing. Returning empty path.");
            return path;
        }
    }

    // To work in the grid of no-fly zones is needed to convert the input points from cartesian (PointStamped) to cells in the grid.
    unsigned int initial_point_in_grid_x =(unsigned int) ( (_initial_point_stamped.point.x - KML_parser_from_path_planner_.min_x_ + x_cell_width_)/x_cell_width_ + 0.00000001);
    unsigned int initial_point_in_grid_y =(unsigned int) ( (_initial_point_stamped.point.y - KML_parser_from_path_planner_.min_y_ + y_cell_width_)/y_cell_width_ + 0.00000001);
    unsigned int final_point_in_grid_x = (unsigned int) ( (_final_point_stamped.point.x - KML_parser_from_path_planner_.min_x_ + x_cell_width_)/x_cell_width_ + 0.00000001);
    unsigned int final_point_in_grid_y = (unsigned int) ( (_final_point_stamped.point.y - KML_parser_from_path_planner_.min_y_ + y_cell_width_)/y_cell_width_ + 0.00000001);
    // The initial and final points in grid are saturated to the size of the no-fly zone matrix:
    if (initial_point_in_grid_x>=no_fly_zones_[0].size()) initial_point_in_grid_x=no_fly_zones_[0].size()-1;
    if (initial_point_in_grid_y>=no_fly_zones_[0].size()) initial_point_in_grid_y=no_fly_zones_[0].size()-1;
    if (final_point_in_grid_x>=no_fly_zones_[0].size()) final_point_in_grid_x=no_fly_zones_[0].size()-1;
    if (final_point_in_grid_y>=no_fly_zones_[0].size()) final_point_in_grid_y=no_fly_zones_[0].size()-1;

    if ( (no_fly_zones_[initial_point_in_grid_y][initial_point_in_grid_x]==1)||(no_fly_zones_[final_point_in_grid_y][final_point_in_grid_x]==1) ) {
        if ( (no_fly_zones_[initial_point_in_grid_y][initial_point_in_grid_x]==1)&&(no_fly_zones_[final_point_in_grid_y][final_point_in_grid_x]==1) ) ROS_ERROR("Path Planner: initial and final points inside obstacles. Returning empty path.");
        else if ( no_fly_zones_[initial_point_in_grid_y][initial_point_in_grid_x]==1 ) ROS_ERROR("Path Planner: initial point inside an obstacle. Returning empty path.");
        else if ( no_fly_zones_[final_point_in_grid_y][final_point_in_grid_x]==1 )     ROS_ERROR("Path Planner: final point inside an obstacle. Returning empty path.");
        std::cout << "Problem of too low grid resolution?" << std::endl << "Printing initial point: " << _initial_point_stamped << std::endl << "Printing final point: " << _final_point_stamped << std::endl;        
        return path;    // if the initial or final point is inside an obstacle, finish the algorithm as no possible path can be found (return empty path).
    }

    if ( (initial_point_in_grid_x==final_point_in_grid_x) && (initial_point_in_grid_y==final_point_in_grid_y) ) {
        path.push_back(_final_point_stamped);
        path_distance_ = sqrt( pow(_final_point_stamped.point.x-_initial_point_stamped.point.x,2) + pow(_final_point_stamped.point.y-_initial_point_stamped.point.y,2) + pow(_final_point_stamped.point.z-_initial_point_stamped.point.z,2) );
        path_flat_distance_ = sqrt( pow(_final_point_stamped.point.x-_initial_point_stamped.point.x,2) + pow(_final_point_stamped.point.y-_initial_point_stamped.point.y,2) );
        return path;    // if the initial and final points are in the same cell (obstacle-free), finish the algorithm and return the final point.
    }

    // A* algorithm implemented with a grid of no-fly zones. Each cell has a value of 0 or 1 if it's free space or no-fly zone, respectively.
    unsigned int rows_no_fly_zone = no_fly_zones_.size();                  // Number of rows of the grid.
    unsigned int columns_no_fly_zone = no_fly_zones_[0].size();               // NMumber of columns of the grid.

    std::map<unsigned int,CellInfo> cell;   // Map whose keys are the cell identifiers and values are "CellInfo" classes (which contains necessary info for the A* algorithm) 
    unsigned int open_set_counter = 0 ;

    unsigned int current = (initial_point_in_grid_x +1) + initial_point_in_grid_y*columns_no_fly_zone; // Cell identifier of the initial position.
    // Example of cell identifiers with a grid of rows_no_fly_zone=3 and columns_no_fly_zone=4 columns (origin of x and y of value 0 in the top left corner):
    //      x=0    x=1    x=2    x=3
    // y=0   1      2      3      4
    // y=1   5      6      7      8
    // y=2   9     10     11     12

    // The first cell to be opened is the initial position, so it's added to the cell map with it's identifier.
    cell[current].x = initial_point_in_grid_x;
    cell[current].y = initial_point_in_grid_y;
    cell[current].came_from = 0;        // Initial position came from nowhere
    cell[current].g_score = 0;          // Cost null from initial position to initial position.

    // This is the calculation of the heuristic and f_score. In the initial position, all the cost is heuristic (g_score = 0). The heuristic is considered as the straight line between the current cell and the final position, ignoring obstacles.
    unsigned int dif_x = final_point_in_grid_x > cell[current].x ? final_point_in_grid_x - cell[current].x : cell[current].x - final_point_in_grid_x ;
    unsigned int dif_y =  final_point_in_grid_y > cell[current].y ? final_point_in_grid_y - cell[current].y : cell[current].y - final_point_in_grid_y ;
    unsigned int min_dif = dif_x > dif_y ? dif_y : dif_x;
    unsigned int dif_x_y = dif_x > dif_y ? dif_x-dif_y : dif_y-dif_x;
    unsigned int heuristic_A_star = dif_x_y*10 + min_dif*14 ;           // Cells in diagonal considered to cost 14 and cells in horizontal or vertical are considered to cost 10.
    cell[current].f_score = cell[current].g_score + heuristic_A_star;

    open_set_counter++;             // A new cell (initial position) has been opened.

    while ( open_set_counter ) {    // If open_set_counter reaches zero without path then the path doesn't exist. An empty path will be returned, with a path_distance_ of "infinity".

        // Search of the open cell with the minimum f_score. This cell will be the current cell in this iteration.
        unsigned int minimum_f_score = std::numeric_limits<unsigned int>::max();
        for ( std::map<unsigned int,CellInfo>::iterator it = cell.begin(); it != cell.end(); it++ ) { // Iterate the whole map
            if ( ( it->second.closed == false ) && ( it->second.f_score < minimum_f_score ) ) {       // ... and if a lower f_score is found, save it and his cell identifier.
                minimum_f_score = it->second.f_score;
                current = it->first;
            }
        }

        // If current cell is the final position, the path has been found, so return path.
        if ( (cell[current].x==final_point_in_grid_x) && (cell[current].y==final_point_in_grid_y) ) {
            // Create the path (in term of cells of the grid or matrix) from the final position cell to the starting position cell.
            geometry_msgs::PointStamped point_from_path;
            point_from_path.point.x = cell[current].x;
            point_from_path.point.y = cell[current].y;
            path_cells.push_back(point_from_path);     // Push the last waypoint into the path vector...
            current = cell[current].came_from;         // ... and moves into his father cell.
            while (current != 0) {
                point_from_path.point.x = cell[current].x;
                point_from_path.point.y = cell[current].y;
                path_cells.push_back(point_from_path);     // Push the last waypoint into the path_cells vector...
                current = cell[current].came_from;         // ... and moves into his father cell.
            }

            std::reverse(path_cells.begin(),path_cells.end());    // The path_cells right now is reversed (last waypoint first), so the order is corrected.

            // path_cells has too much waypoints right now. Waypoints where there isn't a change of direction are annotated as redundant elements.
            current = 1;
            std::vector<unsigned int> redundant_elements;
            while ( not( ((unsigned int)path_cells[current].point.y == final_point_in_grid_y ) && ((unsigned int)path_cells[current].point.x == final_point_in_grid_x ) ) ) {
                if ( (path_cells[current-1].point.x-path_cells[current].point.x==path_cells[current].point.x-path_cells[current+1].point.x) && (path_cells[current-1].point.y-path_cells[current].point.y==path_cells[current].point.y-path_cells[current+1].point.y) )
                    redundant_elements.push_back(current);
                current++;
            }
            std::vector<unsigned int> elements_to_be_erased;

            if ( _movement_pattern ) {    // _movement_pattern == 1 means that the agent moves only horizontally, vertically and diagonally (the movement contemplated in the grid), in other words angles multiple of 45º. Just the redundant elements will be erased, no visibility calculation is needed.
                elements_to_be_erased = redundant_elements;
            } else {                      // _movement_pattern == 0 means that the agent can move in any direction. For this a visibility post-process have to be done inside the path.
                // The elements that are not redundant are the ones that have change of direction.
                std::vector<unsigned int> change_of_direction_elements;
                for ( int i=0; i<path_cells.size(); i++) change_of_direction_elements.push_back(i);
                for ( int i=redundant_elements.size()-1; i>=0; i-- ) change_of_direction_elements.erase(change_of_direction_elements.begin()+redundant_elements[i]);

                // Visibility post-process with the path:
                std::vector<unsigned int> final_path_elements;
                current=0;
                final_path_elements.push_back(change_of_direction_elements[current]);       // Insert the first point always in "final_path_elements"
                unsigned int test_cell;
                while ( current <= change_of_direction_elements.size()-2 ) {      // Visibility loop
                    // std::cout << "current: " << current << std::endl;
                    // std::cout << "change_of_direction_elements: "; for (int h=0; h<change_of_direction_elements.size(); h++) std::cout << change_of_direction_elements[h] << " "; std::cout << std::endl;
                    // std::cout << "final_path_elements: "; for (int h=0; h<final_path_elements.size(); h++) std::cout << final_path_elements[h] << " "; std::cout << std::endl;
                    if ( current < change_of_direction_elements.size()-2 ) {
                        bool collision_flag;
                        for ( test_cell=change_of_direction_elements[current+2]; test_cell>change_of_direction_elements[current+1]; test_cell-- ) {      // change_of_direction_elements[test_cell] is contained between change_of_direction_elements[current+2] and change_of_direction_elements[current+1]. A visibility loop is done in the segment between change_of_direction_elements[test_cell] and change_of_direction_elements[current].
                            // std::cout << "test_cell: " << test_cell << std::endl;

                            unsigned int first_x_segment_cell = (unsigned int) path_cells[change_of_direction_elements[current]].point.x;
                            unsigned int first_y_segment_cell = (unsigned int) path_cells[change_of_direction_elements[current]].point.y;
                            unsigned int last_x_segment_cell = (unsigned int) path_cells[test_cell].point.x;
                            unsigned int last_y_segment_cell = (unsigned int) path_cells[test_cell].point.y;
                            std::vector< std::pair<double,double> > points_intersections_of_segment_with_grid = calculateIntersectionsOfSegmentWithGrid ( (first_x_segment_cell+0.5)*x_cell_width_ , (first_y_segment_cell+0.5)*y_cell_width_ , (last_x_segment_cell+0.5)*x_cell_width_ , (last_y_segment_cell+0.5)*y_cell_width_ );

                            collision_flag = visibilityCheck (points_intersections_of_segment_with_grid);

                            if ( collision_flag == 0 ) break;    // If the actual segment is valid, break the loop.
                        }
                        if ( collision_flag==0 ) {            // if a shortest arc between current and test_cell has been found by the visibility loop, save it.
                            final_path_elements.push_back(test_cell);
                            if ( test_cell == change_of_direction_elements[current+2] ) {    // The point visible is [current+1]
                                current = current+2;
                            } else {                                                          // The point visible is another
                                change_of_direction_elements[current+1]=test_cell;
                                current++;
                            }
                        } else if ( collision_flag==1 ) {       // if a shortest arc between current and test_cell has NOT been found by the visibility loop, just insert the next change_of_direction_element.
                            final_path_elements.push_back(change_of_direction_elements[current+1]);
                            current++;
                        }
                    } else if ( current == change_of_direction_elements.size()-2 ) {  // Insert the last element if the visibility loop didn't jump the second-to-last.
                        final_path_elements.push_back(change_of_direction_elements[change_of_direction_elements.size()-1]);
                        break;
                    }
                }
                // Those elements that aren't in "final_path_elements" are stored in "elements_to_be_erased".
                for ( int i=0; i<path_cells.size(); i++) elements_to_be_erased.push_back(i);
                for ( int i=final_path_elements.size()-1; i>=0; i-- ) elements_to_be_erased.erase(elements_to_be_erased.begin()+final_path_elements[i]);
            }

            // Finally, the elements to be erased are erased from path_cell.
            for ( int i=elements_to_be_erased.size()-1; i>=0; i-- ) path_cells.erase(path_cells.begin()+elements_to_be_erased[i]);

            // Up until now the path has been calculated with waypoints in the middle of the grid. Now the real trajectory will be calculated, wich will contain the destination point, taking into account the initial position of the robot.
            // First, if the center of the second waypoint cell can be reached from the initial position of the robot without collision (they are in the same horizontal, vertical, or visible) then the first waypoint is ignored. If not, the first waypoint will be ignored too but adding an auxiliar waypoint.
            std::vector< std::pair<double,double> > points_intersections_of_segment_with_grid;
            bool collision_flag;
            unsigned int last_x_segment_cell;
            unsigned int last_y_segment_cell;

            if ( not( (initial_point_in_grid_x==(unsigned int)path_cells[1].point.x) || (initial_point_in_grid_y==(unsigned int)path_cells[1].point.y) ) ) {     // Check if the initial point of the UAV and the second waypoint are in the same horizontal or vertical, if they are doesn't enter in the if.

                points_intersections_of_segment_with_grid = calculateIntersectionsOfSegmentWithGrid ( _initial_point_stamped.point.x-KML_parser_from_path_planner_.min_x_+x_cell_width_ , _initial_point_stamped.point.y-KML_parser_from_path_planner_.min_y_+y_cell_width_ , (((unsigned int) path_cells[1].point.x)+0.5)*x_cell_width_ , (((unsigned int) path_cells[1].point.y)+0.5)*y_cell_width_ );

                collision_flag = visibilityCheck (points_intersections_of_segment_with_grid);  // Check if the initial point of the UAV and the second waypoint are visible.

                if ( collision_flag ) {       // If there isn't a direct (visible) path between the initial point of the UAV and path_cells[1] then an intermediate auxiliar point is inserted between them. This point will be the closest intersection of the segment with the vertical or horizontal from the initial point (the one that reduces most the distance).
                    geometry_msgs::PointStamped first_point_from_path;

                    double horizontal_intersection_initial_x = _initial_point_stamped.point.x-KML_parser_from_path_planner_.min_x_+x_cell_width_;
                    double horizontal_intersection_initial_y = (((unsigned int) path_cells[0].point.y)+0.5)*y_cell_width_  + (horizontal_intersection_initial_x - (((unsigned int) path_cells[0].point.x)+0.5)*x_cell_width_) * ((((unsigned int) path_cells[1].point.y)+0.5)*y_cell_width_ - (((unsigned int) path_cells[0].point.y)+0.5)*y_cell_width_)/((((unsigned int) path_cells[1].point.x)+0.5)*x_cell_width_ - (((unsigned int) path_cells[0].point.x)+0.5)*x_cell_width_); // y value in meters of the intersection of the segment with the vertical line of the grid.
                    double vertical_intersection_initial_y =   _initial_point_stamped.point.y-KML_parser_from_path_planner_.min_y_+y_cell_width_;
                    double vertical_intersection_initial_x =   (((unsigned int) path_cells[0].point.x)+0.5)*x_cell_width_  + (vertical_intersection_initial_y   - (((unsigned int) path_cells[0].point.y)+0.5)*y_cell_width_) * ((((unsigned int) path_cells[1].point.x)+0.5)*x_cell_width_ - (((unsigned int) path_cells[0].point.x)+0.5)*x_cell_width_)/((((unsigned int) path_cells[1].point.y)+0.5)*y_cell_width_ - (((unsigned int) path_cells[0].point.y)+0.5)*y_cell_width_); // x value in meters of the intersection of the segment with the vertical line of the grid.

                    if ( sqrt(pow(horizontal_intersection_initial_x-(((unsigned int) path_cells[1].point.x)+0.5)*x_cell_width_,2)+pow(horizontal_intersection_initial_y-(((unsigned int) path_cells[1].point.y)+0.5)*y_cell_width_,2)) <= sqrt(pow(vertical_intersection_initial_x-(((unsigned int) path_cells[1].point.x)+0.5)*x_cell_width_,2)+pow(vertical_intersection_initial_y-(((unsigned int) path_cells[1].point.y)+0.5)*y_cell_width_,2)) ) {
                        first_point_from_path.point.x = horizontal_intersection_initial_x+KML_parser_from_path_planner_.min_x_-x_cell_width_;
                        first_point_from_path.point.y = horizontal_intersection_initial_y+KML_parser_from_path_planner_.min_y_-y_cell_width_;
                    } else {
                        first_point_from_path.point.x = vertical_intersection_initial_x+KML_parser_from_path_planner_.min_x_-x_cell_width_;
                        first_point_from_path.point.y = vertical_intersection_initial_y+KML_parser_from_path_planner_.min_y_-y_cell_width_;
                    }

                    path.push_back(first_point_from_path);
                }
            }
            // Second, the rest of the trajectory is loaded.
            for ( int i=1; i<=path_cells.size()-2; i++ ) {
                geometry_msgs::PointStamped point_from_path;
                point_from_path.point.x = path_cells[i].point.x * x_cell_width_ + KML_parser_from_path_planner_.min_x_ - x_cell_width_/2;
                point_from_path.point.y = path_cells[i].point.y * y_cell_width_ + KML_parser_from_path_planner_.min_y_ - y_cell_width_/2;
                path.push_back(point_from_path);
            }
            // Finally, if the final position of the robot can be reached without collision from the center of the second-last waypoint cell (they are in the same horizontal, vertical, or visible) then the last waypoint is ignored. If not, the last waypoint will be ignored too but adding an auxiliar waypoint.
            if ( not( (final_point_in_grid_x==(unsigned int)path_cells[path_cells.size()-2].point.x) || (final_point_in_grid_y==(unsigned int)path_cells[path_cells.size()-2].point.y) ) ) {

                points_intersections_of_segment_with_grid = calculateIntersectionsOfSegmentWithGrid ( _final_point_stamped.point.x-KML_parser_from_path_planner_.min_x_+x_cell_width_ , _final_point_stamped.point.y-KML_parser_from_path_planner_.min_y_+y_cell_width_ , (((unsigned int) path_cells[path_cells.size()-2].point.x)+0.5)*x_cell_width_ , (((unsigned int) path_cells[path_cells.size()-2].point.y)+0.5)*y_cell_width_ );

                bool collision_flag = visibilityCheck (points_intersections_of_segment_with_grid);

                if ( collision_flag ) {       // If there isn't a direct (visible) path between the final point and path_cells[path_cells.size()-2] then an intermediate point is inserted between them. This point will be the closest intersection of the segment with the vertical or horizontal from the final point (the one that reduces most the distance).
                    geometry_msgs::PointStamped second_last_point_from_path;

                    double horizontal_intersection_final_x = _final_point_stamped.point.x-KML_parser_from_path_planner_.min_x_+x_cell_width_;
                    double horizontal_intersection_final_y = (((unsigned int) path_cells[path_cells.size()-1].point.y)+0.5)*y_cell_width_  + (horizontal_intersection_final_x - (((unsigned int) path_cells[path_cells.size()-1].point.x)+0.5)*x_cell_width_) * ((((unsigned int) path_cells[path_cells.size()-2].point.y)+0.5)*y_cell_width_ - (((unsigned int) path_cells[path_cells.size()-1].point.y)+0.5)*y_cell_width_)/((((unsigned int) path_cells[path_cells.size()-2].point.x)+0.5)*x_cell_width_ - (((unsigned int) path_cells[path_cells.size()-1].point.x)+0.5)*x_cell_width_); // y value in meters of the intersection of the segment with the vertical line of the grid.
                    double vertical_intersection_final_y =   _final_point_stamped.point.y-KML_parser_from_path_planner_.min_y_+y_cell_width_;
                    double vertical_intersection_final_x =   (((unsigned int) path_cells[path_cells.size()-1].point.x)+0.5)*x_cell_width_  + (vertical_intersection_final_y   - (((unsigned int) path_cells[path_cells.size()-1].point.y)+0.5)*y_cell_width_) * ((((unsigned int) path_cells[path_cells.size()-2].point.x)+0.5)*x_cell_width_ - (((unsigned int) path_cells[path_cells.size()-1].point.x)+0.5)*x_cell_width_)/((((unsigned int) path_cells[path_cells.size()-2].point.y)+0.5)*y_cell_width_ - (((unsigned int) path_cells[path_cells.size()-1].point.y)+0.5)*y_cell_width_); // x value in meters of the intersection of the segment with the vertical line of the grid.

                    if ( sqrt(pow(horizontal_intersection_final_x-(((unsigned int) path_cells[path_cells.size()-2].point.x)+0.5)*x_cell_width_,2)+pow(horizontal_intersection_final_y-(((unsigned int) path_cells[path_cells.size()-2].point.y)+0.5)*y_cell_width_,2)) <= sqrt(pow(vertical_intersection_final_x-(((unsigned int) path_cells[path_cells.size()-2].point.x)+0.5)*x_cell_width_,2)+pow(vertical_intersection_final_y-(((unsigned int) path_cells[path_cells.size()-2].point.y)+0.5)*y_cell_width_,2)) ) {
                        second_last_point_from_path.point.x = horizontal_intersection_final_x+KML_parser_from_path_planner_.min_x_-x_cell_width_;
                        second_last_point_from_path.point.y = horizontal_intersection_final_y+KML_parser_from_path_planner_.min_y_-y_cell_width_;
                    } else {
                        second_last_point_from_path.point.x = vertical_intersection_final_x+KML_parser_from_path_planner_.min_x_-x_cell_width_;
                        second_last_point_from_path.point.y = vertical_intersection_final_y+KML_parser_from_path_planner_.min_y_-y_cell_width_;
                    }

                    path.push_back(second_last_point_from_path);
                }
            }
            path.push_back(_final_point_stamped);

            // Calculation of the path_distance_ and path_flat_distance_ (sum of norms between pairs of waypoints)
            path_flat_distance_ = 0;
            for (current=0; current<path.size()-1; current++) {
                path_flat_distance_ += sqrt(pow(path[current].point.x-path[current+1].point.x,2)+pow(path[current].point.y-path[current+1].point.y,2));
                path[current].point.z = _final_point_stamped.point.z;     // All the waypoints at the height (z) of the final point (final element of path is already final point).
            }
            path_distance_      = path_flat_distance_ + sqrt(pow(path[0].point.x-_initial_point_stamped.point.x,2)+pow(path[0].point.y-_initial_point_stamped.point.y,2)+pow(_final_point_stamped.point.z-_initial_point_stamped.point.z,2));  // All the z movement done reaching the first waypoint.
            path_flat_distance_ = path_flat_distance_ + sqrt(pow(path[0].point.x-_initial_point_stamped.point.x,2)+pow(path[0].point.y-_initial_point_stamped.point.y,2));      // No z movement if supposed flat movement.

            break;  // Terminate the algorithm.

        }   // End of actions when path is found (path creation, visibility post-process and calculation of distance).

        // We have the current cell with the minimum f_score, which isn't the final position. The first thing to do is to close the current cell and decrement the open_set_counter.
        cell[current].closed = true;
        open_set_counter--;

        // For each neighbour cell in the x axis of the grid...
        for ( int x_adjacent=-1; x_adjacent<=1; x_adjacent++) {
            // ... and for each neighbour cell in the y axis of the grid...
            for ( int y_adjacent=-1; y_adjacent<=1; y_adjacent++) {
                if ( (x_adjacent==0) && (y_adjacent==0) ) continue;      // This isn't a neighbour cell, this is the current cell. Ignore it.
                else if ( ((int)cell[current].x+x_adjacent<0) || ((int)cell[current].x+x_adjacent>=columns_no_fly_zone) || ((int)cell[current].y+y_adjacent<0) || ((int)cell[current].y+y_adjacent>=rows_no_fly_zone) ) continue;     // Neighbour cell out of the limits of the grid. Ignore it.
                else if ( no_fly_zones_[cell[current].y+y_adjacent][cell[current].x+x_adjacent]==1 ) continue;                                                                                       // Neighbour cell with obstacles. Ignore it.
                else if ( ((x_adjacent!=0) && (y_adjacent!=0)) && ( (no_fly_zones_[cell[current].y][cell[current].x+x_adjacent]==1) && (no_fly_zones_[cell[current].y+y_adjacent][cell[current].x]==1)) ) continue;  // The line between the current cell and the neighbour cell goes between two obstacles. Ignore it, can't reach the neighbour cell.

                // Calculation of the cell identifier of the neighbour cell.
                unsigned int current_neighbour = cell[current].x+x_adjacent+1 + ( cell[current].y+y_adjacent )*columns_no_fly_zone;

                if ( cell.count(current_neighbour) <= 0 ) {   // The neighbour cell not being found in the cell map means that the neighbour cell is unexplored yet. It has to be inserted to the cell map.
                    cell[current_neighbour].x = cell[current].x + x_adjacent;
                    cell[current_neighbour].y = cell[current].y + y_adjacent;
                    open_set_counter++;                                         // A new cell (neighbour cell) has been opened.
                } else {   // The neighbour cell is found in the cell map, but it's closed. Ignore it.
                    if (cell[current_neighbour].closed == true) continue;
                }

                // If neighbour cell is in diagonal from the current cell, the distance between them is considered to be sqrt(10)~14. If not, the distance is considered to be 10.
                // This distances are considered instead of the real ones because it's faster and there is no difference in convergence.
                unsigned int distance_to_neighbour;
                if ( (x_adjacent==0) || (y_adjacent==0) ) distance_to_neighbour=10;
                else distance_to_neighbour = 14;

                // g_score_test is the g_score that the neighbour cell would have if his parent was the current cell
                unsigned int g_score_test = cell[current].g_score + distance_to_neighbour;

                if ( g_score_test >= cell[current_neighbour].g_score ) continue;     // g_score_test doesn't improve the g_score of the neighbour cell. Ignore it.

                // When the program gets here, it means that the neighbour cell is open and the current cell improves his g_score.
                // So now the neighbour cell is updated.
                cell[current_neighbour].came_from = current;
                cell[current_neighbour].g_score = g_score_test;

                // This is the calculation of the heuristic and f_score. The heuristic is considered as the straight line between the current cell and the final position, ignoring obstacles.
                dif_x = final_point_in_grid_x > cell[current_neighbour].x ? final_point_in_grid_x - cell[current_neighbour].x : cell[current_neighbour].x - final_point_in_grid_x ;
                dif_y =  final_point_in_grid_y > cell[current_neighbour].y ? final_point_in_grid_y - cell[current_neighbour].y : cell[current_neighbour].y - final_point_in_grid_y ;
                min_dif = dif_x > dif_y ? dif_y : dif_x;
                dif_x_y = dif_x > dif_y ? dif_x-dif_y : dif_y-dif_x;
                heuristic_A_star = dif_x_y*10 + min_dif*14 ;           // Cells in diagonal considered to cost 14 and cells in horizontal or vertical are considered to cost 10.
                cell[current_neighbour].f_score = g_score_test + heuristic_A_star ;
            }
        }
    }

    if ( _show_results ) {    
        /////////////////////////////// Show results in the terminal. ///////////////////////////////
        t_end = clock();
        double seconds = ((float)(t_end-t_begin))/CLOCKS_PER_SEC;

        std::vector<std::vector<char>> visual_matrix_for_debug;
        for (unsigned int i=0;i<rows_no_fly_zone;i++) {
            std::vector<char> row;
            for (unsigned int j=0;j<columns_no_fly_zone;j++) {
                if ( no_fly_zones_[i][j] )   row.push_back('1');
                else    row.push_back(' ');
            }
            visual_matrix_for_debug.push_back(row);
        }
        visual_matrix_for_debug[initial_point_in_grid_y][initial_point_in_grid_x]='A';    // Should be overwritten by the path (but same leter, would look the same)
        visual_matrix_for_debug[final_point_in_grid_y][final_point_in_grid_x]='Z';

        for (unsigned int i=0;i<path_cells.size();i++) {
            if ( not(((unsigned int)path_cells[i].point.y == final_point_in_grid_y ) && ((unsigned int)path_cells[i].point.x == final_point_in_grid_x ) ) )
                visual_matrix_for_debug[(unsigned int)path_cells[i].point.y][(unsigned int)path_cells[i].point.x]=i+'A';
        }

        std::cout << std::endl << "Representation of the obstacles (no-fly zones) and trajectory:" << std::endl;
        for (int i=rows_no_fly_zone-1;i>=0;i--) {    // IMPORTANT: visual matrix represented upside-down!! Thats because the y axis is backwards when treated as matrix.
            for (int j=0;j<columns_no_fly_zone;j++) {
                std::cout << visual_matrix_for_debug[i][j] << ' ';
            }
            std::cout << std::endl;
        }

        std::cout << std::endl  << "no_fly_zones_ size: " << no_fly_zones_.size() << " x " << no_fly_zones_[0].size() << std::endl;
        std::cout << "x_cell_width_: " << x_cell_width_ << "  y_cell_width_: " << y_cell_width_ << std::endl << std::endl;

        double distance = getDistance();
        std::cout << "Distance of path: " << distance << std::endl << std::endl;

        std::cout << "Computation time for getPath: " << seconds << " seconds." << std::endl << std::endl;

        std::cout << "Real trajectory: " << std::endl; for (int i=0; i<path.size(); i++) std::cout << path[i] << std::endl; std::cout << std::endl;
        std::cout << "Grid trajectory: " << std::endl; for (int i=0; i<path_cells.size(); i++) std::cout << path_cells[i] << std::endl; std::cout << std::endl;  // << std::setprecision(10)
        /////////////////////////////// Show results in the terminal. ///////////////////////////////

        ///////////////////////////// DEBUG, plot grid, obstacles and path. /////////////////////////////
#ifdef DEBUG_MODE
        // plot the intersections of the sides of the obstacle's polygons with the grid:
        for (int i=0; i<KML_parser_from_path_planner_.no_fly_zones_cartesian_.size(); i++) {           // for every no-fly zone polygon...
            for (int j=0; j<KML_parser_from_path_planner_.no_fly_zones_cartesian_[i].size()-1; j++) {  // The pairs of points [j] to [j+1] from the polygon are checked forming a segment. The intersection of that segment with the lines of the grid will indicate the cells with obstacles.
                std::vector< std::pair<double,double> > points_intersections_of_segment_with_grid = calculateIntersectionsOfSegmentWithGrid (KML_parser_from_path_planner_.no_fly_zones_cartesian_[i][j].x - KML_parser_from_path_planner_.min_x_ + x_cell_width_ , KML_parser_from_path_planner_.no_fly_zones_cartesian_[i][j].y - KML_parser_from_path_planner_.min_y_ + y_cell_width_ , KML_parser_from_path_planner_.no_fly_zones_cartesian_[i][j+1].x - KML_parser_from_path_planner_.min_x_ + x_cell_width_ , KML_parser_from_path_planner_.no_fly_zones_cartesian_[i][j+1].y - KML_parser_from_path_planner_.min_y_ + y_cell_width_);

                std::vector<double> x_plot, y_plot;
                for (int k=0; k<points_intersections_of_segment_with_grid.size(); k++) { x_plot.push_back(points_intersections_of_segment_with_grid[k].first); y_plot.push_back(points_intersections_of_segment_with_grid[k].second); }
                plt::plot(x_plot,y_plot,"xg-");
            }
        }
        for (int i=0; i<KML_parser_from_path_planner_.private_areas_cartesian_.size(); i++) {          // for every no-fly zone polygon...
            for (int j=0; j<KML_parser_from_path_planner_.private_areas_cartesian_[i].size()-1; j++) { // The pairs of points [j] to [j+1] from the polygon are checked forming a segment. The intersection of that segment with the lines of the grid will indicate the cells with obstacles.
                std::vector< std::pair<double,double> > points_intersections_of_segment_with_grid = calculateIntersectionsOfSegmentWithGrid (KML_parser_from_path_planner_.private_areas_cartesian_[i][j].x - KML_parser_from_path_planner_.min_x_ + x_cell_width_ , KML_parser_from_path_planner_.private_areas_cartesian_[i][j].y - KML_parser_from_path_planner_.min_y_ + y_cell_width_ , KML_parser_from_path_planner_.private_areas_cartesian_[i][j+1].x - KML_parser_from_path_planner_.min_x_ + x_cell_width_ , KML_parser_from_path_planner_.private_areas_cartesian_[i][j+1].y - KML_parser_from_path_planner_.min_y_ + y_cell_width_);

                std::vector<double> x_plot, y_plot;
                for (int k=0; k<points_intersections_of_segment_with_grid.size(); k++) { x_plot.push_back(points_intersections_of_segment_with_grid[k].first); y_plot.push_back(points_intersections_of_segment_with_grid[k].second); }
                plt::plot(x_plot,y_plot,"xg-");
            }
        }

        // plot vertical lines of the grid:
        for (int k=0; k<=no_fly_zones_[0].size();k++) {
            std::vector<double> x_plot, y_plot;
            x_plot.push_back(k*x_cell_width_); x_plot.push_back(k*x_cell_width_);
            y_plot.push_back(0); y_plot.push_back(no_fly_zones_.size()*y_cell_width_);
            plt::plot(x_plot,y_plot,"b");
        }

        // plot horizontal lines of the grid:
        for (int k=0; k<=no_fly_zones_.size();k++) {
            std::vector<double> x_plot, y_plot;
            x_plot.push_back(0); x_plot.push_back(no_fly_zones_[0].size()*x_cell_width_);
            y_plot.push_back(k*y_cell_width_); y_plot.push_back(k*y_cell_width_);
            plt::plot(x_plot,y_plot,"b");
        }

        // fill the inside of the cells with obstacles with a cross:
        for (int i=0;i<no_fly_zones_.size();i++) {
            for (int j=0;j<no_fly_zones_[0].size();j++) {
                if (no_fly_zones_[i][j]==1) {
                    double x_1=j*x_cell_width_;     double x_2=(j+1)*x_cell_width_;
                    double y_1=i*y_cell_width_;     double y_2=(i+1)*y_cell_width_;
                    std::vector<double> x_plot, y_plot;
                    x_plot.push_back(x_1);          x_plot.push_back(x_2);
                    y_plot.push_back(y_1);          y_plot.push_back(y_2);
                    plt::plot(x_plot, y_plot, "r");
                    x_plot.clear(), y_plot.clear();
                    x_plot.push_back(x_1);          x_plot.push_back(x_2);
                    y_plot.push_back(y_2);          y_plot.push_back(y_1);
                    plt::plot(x_plot, y_plot, "r");
                }
            }
        }

        if ( path.size()!=0 ) {  // if path exist draw it.
            // Real trajectory:
            std::vector<double> x_plot, y_plot;
            x_plot.push_back(_initial_point_stamped.point.x - KML_parser_from_path_planner_.min_x_ + x_cell_width_);
            y_plot.push_back(_initial_point_stamped.point.y - KML_parser_from_path_planner_.min_y_ + y_cell_width_);
            for (int i=0; i<path.size(); i++) {
                x_plot.push_back(path[i].point.x - KML_parser_from_path_planner_.min_x_ + x_cell_width_);
                y_plot.push_back(path[i].point.y - KML_parser_from_path_planner_.min_y_ + y_cell_width_);
            }
            plt::plot(x_plot, y_plot, "og-");
            x_plot.clear(), y_plot.clear();

            // Grid trajectory:
            x_plot.push_back(_initial_point_stamped.point.x - KML_parser_from_path_planner_.min_x_ + x_cell_width_);
            y_plot.push_back(_initial_point_stamped.point.y - KML_parser_from_path_planner_.min_y_ + y_cell_width_);
            for (int i=0; i<path_cells.size(); i++) {
                x_plot.push_back(path_cells[i].point.x*x_cell_width_+x_cell_width_/2);
                y_plot.push_back(path_cells[i].point.y*y_cell_width_+y_cell_width_/2);
            }
            x_plot.push_back(_final_point_stamped.point.x - KML_parser_from_path_planner_.min_x_ + x_cell_width_);
            y_plot.push_back(_final_point_stamped.point.y - KML_parser_from_path_planner_.min_y_ + y_cell_width_);
            plt::plot(x_plot, y_plot, "xy-");

        }

        // Draw (show) everything:
        plt::show();
#endif
        ///////////////////////////// DEBUG, plot grid, obstacles and path. /////////////////////////////
    }

    if ( path.size()==0 ) {
        ROS_WARN("Path Planner: No path possible between the initial and final points, even though those two are obstacle-free. Returning empty path. Problem of too low grid resolution?");
        std::cout << "Printing initial point: " << _initial_point_stamped << std::endl << "Printing final point: " << _final_point_stamped << std::endl;
    }

    return path;
}   // end method "getPath"



// "getPath" method that estimates the time in each waypoint given the speed of the drone: A* algorithm implementation that returns a path for one robot from his initial position to the end position.
std::vector<geometry_msgs::PointStamped> PathPlanner::getPathWithTimePredictions(const geometry_msgs::PointStamped& _initial_point_stamped, const geometry_msgs::PointStamped& _final_point_stamped, int _full_speed_xy, int _full_speed_z_down, int _full_speed_z_up, bool _movement_pattern, bool _show_results) {

    std::vector<geometry_msgs::PointStamped> path_stamped = getPath(_initial_point_stamped, _final_point_stamped, _movement_pattern, _show_results);

    if ( _final_point_stamped.header.stamp.sec < _initial_point_stamped.header.stamp.sec ) {
        std::cout << "PathPlanner, getPathWithTimePredictions. Error: final point time before than initial point time." << std::endl;
    } else if ( path_stamped.size() > 1 ) {    // If size==0 the path already has the time at the final (only) waypoint becayse the final point stamped is directly inserted.

        // All the waypoints of the path has the height (z) of the final point. If the two points (initial and final) have different heights, that height difference will be travelled between the initial point and the first waypoint.

        // Flat movement:
        float distance_xy = getFlatDistance();
        float first_distance_xy = sqrt(pow(path_stamped[0].point.x-_initial_point_stamped.point.x,2)+pow(path_stamped[0].point.y-_initial_point_stamped.point.y,2));  // Distance from the initial point to the first waypoint of the path.

        int moving_time_xy = distance_xy/_full_speed_xy; 
        int first_moving_time_xy = first_distance_xy/_full_speed_xy;


        // Vertical movement:
        float height_difference_z = _final_point_stamped.point.z - _initial_point_stamped.point.z;      // Difference of height between the two points.

        int moving_time_z = height_difference_z >= 0 ? height_difference_z/_full_speed_z_up : abs(height_difference_z)/_full_speed_z_down;


        if ( first_moving_time_xy >= moving_time_z ) {
            // Horizontal moving time greater than vertical moving time between the initial point and the first waypoint. Ignore vertical time.
            int time_difference = _final_point_stamped.header.stamp.sec - _initial_point_stamped.header.stamp.sec;

            double aux_path_flat_distance = sqrt(pow(path_stamped[0].point.x-_initial_point_stamped.point.x,2)+pow(path_stamped[0].point.y-_initial_point_stamped.point.y,2));
            for (int i=0; i<path_stamped.size()-1; i++) {
                path_stamped[i].header.stamp.sec = _initial_point_stamped.header.stamp.sec + time_difference * aux_path_flat_distance/distance_xy;                 // Estimate time by distance done at the waypoint in proportion with the total and the time difference.
                aux_path_flat_distance += sqrt(pow(path_stamped[i].point.x-path_stamped[i+1].point.x,2)+pow(path_stamped[i].point.y-path_stamped[i+1].point.y,2)); // Update distance done at the waypoint.
            }
        } else {
            // Vertical moving time greater than horizontal moving time between the initial point and the first waypoint. Ignore horizontal time only in that first waypoint.
            path_stamped[0].header.stamp.sec = _initial_point_stamped.header.stamp.sec + moving_time_z;

            int time_difference = _final_point_stamped.header.stamp.sec - path_stamped[0].header.stamp.sec;

            double aux_path_flat_distance = sqrt(pow(path_stamped[1].point.x-path_stamped[0].point.x,2)+pow(path_stamped[1].point.y-path_stamped[0].point.y,2));
            for (int i=1; i<path_stamped.size()-1; i++) {
                path_stamped[i].header.stamp.sec = path_stamped[0].header.stamp.sec + time_difference * aux_path_flat_distance/(distance_xy-first_distance_xy);    // Estimate time by distance done at the waypoint in proportion with the total and the time difference.
                aux_path_flat_distance += sqrt(pow(path_stamped[i].point.x-path_stamped[i+1].point.x,2)+pow(path_stamped[i].point.y-path_stamped[i+1].point.y,2)); // Update distance done at the waypoint.
            }
        }

    }

    return path_stamped;
}   // end overloaded method "getPathWithTimePredictions"



// "getPathWithTimePredictionsAndInitialPoint" method, exactly the same as the one before but including the initial point with its time.
std::vector<geometry_msgs::PointStamped> PathPlanner::getPathWithTimePredictionsAndInitialPoint(const geometry_msgs::PointStamped& _initial_point_stamped, const geometry_msgs::PointStamped& _final_point_stamped, int _full_speed_xy, int _full_speed_z_down, int _full_speed_z_up, bool _movement_pattern, bool _show_results) {

    std::vector<geometry_msgs::PointStamped> path_stamped = getPathWithTimePredictions(_initial_point_stamped, _final_point_stamped, _full_speed_xy, _full_speed_z_down, _full_speed_z_up, _movement_pattern, _show_results);

    path_stamped.insert(path_stamped.begin(), _initial_point_stamped);

    return path_stamped;
}   // end overloaded method "getPathWithTimePredictionsAndInitialPoint"



// "getPath" method overloaded with Point32 instead of PointStamped: A* algorithm implementation that returns a path for one robot from his initial position to the end position.
std::vector<geometry_msgs::Point32> PathPlanner::getPath(const geometry_msgs::Point32& _initial_point, const geometry_msgs::Point32& _final_point, bool _movement_pattern, bool _show_results) {

    std::vector<geometry_msgs::Point32> path_to_return;

    geometry_msgs::PointStamped _initial_point_stamped, _final_point_stamped;
    _initial_point_stamped.point.x = _initial_point.x;
    _initial_point_stamped.point.y = _initial_point.y;
    _initial_point_stamped.point.z = _initial_point.z;
    _final_point_stamped.point.x = _final_point.x;
    _final_point_stamped.point.y = _final_point.y;
    _final_point_stamped.point.z = _final_point.z;
    
    std::vector<geometry_msgs::PointStamped> path_stamped = getPath(_initial_point_stamped, _final_point_stamped, _movement_pattern, _show_results);

    for (int i=0; i<path_stamped.size(); i++) {
        geometry_msgs::Point32 current_point_32;
        current_point_32.x = path_stamped[i].point.x;
        current_point_32.y = path_stamped[i].point.y;
        current_point_32.z = path_stamped[i].point.z;

        path_to_return.push_back(current_point_32);
    }

    return path_to_return;
}   // end overloaded method "getPath"



// "getPath" method but overloaded with Point instead of PointStamped: A* algorithm implementation that returns a path for one robot from his initial position to the end position.
std::vector<geometry_msgs::Point> PathPlanner::getPath(const geometry_msgs::Point& _initial_point, const geometry_msgs::Point& _final_point, bool _movement_pattern, bool _show_results) {

    std::vector<geometry_msgs::Point> path_to_return;

    geometry_msgs::PointStamped _initial_point_stamped, _final_point_stamped;
    _initial_point_stamped.point.x = _initial_point.x;
    _initial_point_stamped.point.y = _initial_point.y;
    _initial_point_stamped.point.z = _initial_point.z;
    _final_point_stamped.point.x = _final_point.x;
    _final_point_stamped.point.y = _final_point.y;
    _final_point_stamped.point.z = _final_point.z;
    
    std::vector<geometry_msgs::PointStamped> path_stamped = getPath(_initial_point_stamped, _final_point_stamped, _movement_pattern, _show_results);

    for (int i=0; i<path_stamped.size(); i++) {
        geometry_msgs::Point current_point;
        current_point.x = path_stamped[i].point.x;
        current_point.y = path_stamped[i].point.y;
        current_point.z = path_stamped[i].point.z;

        path_to_return.push_back(current_point);
    }

    return path_to_return;
}   // end overloaded method "getPath"



// Brief method that returns the cost (distance) of the path
double PathPlanner::getDistance() {
    return path_distance_;
}   // end method "getDistance"



// Brief method that returns the cost (distance) of the path supposed flat
double PathPlanner::getFlatDistance() {
    return path_flat_distance_;
}   // end method "getFlatDistance"



std::vector< std::pair<double,double> > PathPlanner::calculateIntersectionsOfSegmentWithGrid (double first_absolute_point_of_segment_x, double first_absolute_point_of_segment_y, double last_absolute_point_of_segment_x, double last_absolute_point_of_segment_y) {  // This method calculate the intersections of a segment with the lines of the grid, this will be necessary the next two methods.
// IMPORTANT: the inputs are the (x, y) values of the two points that defines a segment but WITH THE COORDINATE ORIGIN CHANGED SO (0,0) IS THE ORIGIN OF THE GRID. In other words, a r.
    std::vector< std::pair<double,double> > points_intersections_of_segment_with_grid;         // Here the intersections of the actual segment with the grid will be stored.

    unsigned int first_cell_of_segment_x = (unsigned int) ( first_absolute_point_of_segment_x/x_cell_width_ +0.00000001 );    // number of cell in the x axis of the point [j] (+0.00000001 to avoid numerical problems)
    unsigned int first_cell_of_segment_y = (unsigned int) ( first_absolute_point_of_segment_y/y_cell_width_ +0.00000001 );    // number of cell in the y axis of the point [j] (+0.00000001 to avoid numerical problems)

    unsigned int last_cell_of_segment_x = (unsigned int) ( last_absolute_point_of_segment_x/x_cell_width_ +0.00000001 );    // number of cell in the x axis of the point [j+1] (+0.00000001 to avoid numerical problems)
    unsigned int last_cell_of_segment_y = (unsigned int) ( last_absolute_point_of_segment_y/y_cell_width_ +0.00000001 );    // number of cell in the y axis of the point [j+1] (+0.00000001 to avoid numerical problems)

    int separation_x_cells_segment = last_cell_of_segment_x - first_cell_of_segment_x;
    int separation_y_cells_segment = last_cell_of_segment_y - first_cell_of_segment_y;

    std::pair <double,double> first_segment_point (first_absolute_point_of_segment_x, first_absolute_point_of_segment_y);
    points_intersections_of_segment_with_grid.push_back(first_segment_point);                  // The first point is stored as the first intersection.

    if ( separation_x_cells_segment > 0 )                    // if the segment goes from left to right (separation different than zero).
        for (int k=1; k<=separation_x_cells_segment; k++) {
            double actual_segment_intersection_x = (first_cell_of_segment_x + k) * x_cell_width_;   // x value in meters of the intersection of the segment with the actual vertical line of the grid.
            double actual_segment_intersection_y = first_absolute_point_of_segment_y + (actual_segment_intersection_x - first_absolute_point_of_segment_x) * (last_absolute_point_of_segment_y - first_absolute_point_of_segment_y)/(last_absolute_point_of_segment_x - first_absolute_point_of_segment_x); // y value in meters of the intersection of the segment with the vertical line of the grid.
            std::pair <double,double> actual_segment_intersection ( actual_segment_intersection_x , actual_segment_intersection_y );
            points_intersections_of_segment_with_grid.push_back(actual_segment_intersection);
        }
    else if ( separation_x_cells_segment < 0 )               // else if the segment goes from right to left (separation different than zero).
        for (int k=0; k>=separation_x_cells_segment+1; k--) {
            double actual_segment_intersection_x = (first_cell_of_segment_x + k) * x_cell_width_;   // x value in meters of the intersection of the segment with the actual vertical line of the grid.
            double actual_segment_intersection_y = first_absolute_point_of_segment_y + (actual_segment_intersection_x - first_absolute_point_of_segment_x) * (last_absolute_point_of_segment_y - first_absolute_point_of_segment_y)/(last_absolute_point_of_segment_x - first_absolute_point_of_segment_x); // y value in meters of the intersection of the segment with the vertical line of the grid.
            std::pair <double,double> actual_segment_intersection ( actual_segment_intersection_x , actual_segment_intersection_y );
            points_intersections_of_segment_with_grid.push_back(actual_segment_intersection);   
        }

    if ( separation_y_cells_segment > 0 )                    // if the segment goes from top to bottom (separation different than zero).
        for (int k=1; k<=separation_y_cells_segment; k++) {
            double actual_segment_intersection_y = (first_cell_of_segment_y + k) * y_cell_width_;   // y value in meters of the intersection of the segment with the actual horizontal line of the grid.
            double actual_segment_intersection_x = first_absolute_point_of_segment_x + (actual_segment_intersection_y - first_absolute_point_of_segment_y) * (last_absolute_point_of_segment_x - first_absolute_point_of_segment_x)/(last_absolute_point_of_segment_y - first_absolute_point_of_segment_y); // x value in meters of the intersection of the segment with the horizontal line of the grid.
            std::pair <double,double> actual_segment_intersection ( actual_segment_intersection_x , actual_segment_intersection_y );
            points_intersections_of_segment_with_grid.push_back(actual_segment_intersection);
        }
    else if ( separation_y_cells_segment < 0 )               // else if the segment goes from top to bottom (separation different than zero).
        for (int k=0; k>=separation_y_cells_segment+1; k--) {
            double actual_segment_intersection_y = (first_cell_of_segment_y + k) * y_cell_width_;   // y value in meters of the intersection of the segment with the actual horizontal line of the grid.
            double actual_segment_intersection_x = first_absolute_point_of_segment_x + (actual_segment_intersection_y - first_absolute_point_of_segment_y) * (last_absolute_point_of_segment_x - first_absolute_point_of_segment_x)/(last_absolute_point_of_segment_y - first_absolute_point_of_segment_y); // x value in meters of the intersection of the segment with the horizontal line of the grid.
            std::pair <double,double> actual_segment_intersection ( actual_segment_intersection_x , actual_segment_intersection_y );
            points_intersections_of_segment_with_grid.push_back(actual_segment_intersection);
        }

    std::pair <double,double> last_segment_point (last_absolute_point_of_segment_x, last_absolute_point_of_segment_y);
    points_intersections_of_segment_with_grid.push_back(last_segment_point);       // The last point is stored as the last intersection.

    // Now is time to sort all the unsorted intersection points of the segment with the grid. The "if" is to avoid problems when the segment is perfectly horizontal or vertical.
    if ( points_intersections_of_segment_with_grid[0].first == points_intersections_of_segment_with_grid[points_intersections_of_segment_with_grid.size()-1].first )  // if all points with the same x value
        std::sort(points_intersections_of_segment_with_grid.begin(),points_intersections_of_segment_with_grid.end(), [](const std::pair<double, double> &left, const std::pair<double, double> &right) {
            return left.second < right.second;
        });     // Sort the "points_intersections_of_segment_with_grid" vector from minimum to maximum y value (the third parameter is a custom comparator to sort the vector of pairs).
    else
        std::sort(points_intersections_of_segment_with_grid.begin(),points_intersections_of_segment_with_grid.end(), [](const std::pair<double, double> &left, const std::pair<double, double> &right) {
            return left.first < right.first;
        });     // Sort the "points_intersections_of_segment_with_grid" vector from minimum to maximum x value (the third parameter is a custom comparator to sort the vector of pairs).

    return points_intersections_of_segment_with_grid;
}   // end method "calculateIntersectionsOfSegmentWithGrid"



void PathPlanner::fillCellsWithObstacles (const std::vector< std::pair<double,double> >& points_intersections_of_segment_with_grid) {   // This method fill of obstacles the cells of a segment, receiving as input the intersections of the segment with the grid.
    // "points_intersections_of_segment_with_grid" now contains in a sorted way all the intersections of the segment with the grid and the beginning and the end of the segment.
    // Now the middle point between pairs of points in "points_intersections_of_segment_with_grid" are calculated. This middle points will represent cells with obstacles.
    // Then, the cell of each "point_inside_cell" is calculated and set as an obstacle in the no_fly_zones_ matrix.
    for (int k=0; k<=points_intersections_of_segment_with_grid.size()-2; k++) {
        double point_inside_cell_x = (points_intersections_of_segment_with_grid[k].first + points_intersections_of_segment_with_grid[k+1].first)/2;   // Middle point (x axis)
        double point_inside_cell_y = (points_intersections_of_segment_with_grid[k].second + points_intersections_of_segment_with_grid[k+1].second)/2; // Middle point (y axis)

        unsigned int x_cell = (unsigned int) (point_inside_cell_x/x_cell_width_ +0.00000001);  // Cell location of "point_inside_cell" in the x axis (+ 0.00000001 to avoid numerical problems)
        unsigned int y_cell = (unsigned int) (point_inside_cell_y/y_cell_width_ +0.00000001);  // Cell location of "point_inside_cell" in the y axis (+ 0.00000001 to avoid numerical problems)

        // Borders of the map should always be obstacle-free. The following is done to force that always:
        if (x_cell<=0) x_cell=1;
        else if (x_cell>=no_fly_zones_[0].size()-1) x_cell=no_fly_zones_[0].size()-2;
        if (y_cell<=0) y_cell=1;
        else if (y_cell>=no_fly_zones_.size()-1) y_cell=no_fly_zones_.size()-2;

        // Finally, that cell is set to 1 in the no_fly_zones_ matrix.
        no_fly_zones_[y_cell][x_cell] = 1;
    }

    // Although improbable, sometimes the last point of the segment is in a grid corner. In this case is necessary to explicitly assign the obstacle to that corner:
    double x_cell_double = points_intersections_of_segment_with_grid[points_intersections_of_segment_with_grid.size()-1].first/x_cell_width_;
    double y_cell_double = points_intersections_of_segment_with_grid[points_intersections_of_segment_with_grid.size()-1].second/y_cell_width_;
    if ( x_cell_double-(unsigned int)x_cell_double==0 && y_cell_double-(unsigned int)y_cell_double==0 ) {    // if double - (unsigned int) equals to cero for both (x,y) means that the point it's in a grid corner.
        unsigned int x_cell = (unsigned int)x_cell_double;
        unsigned int y_cell = (unsigned int)y_cell_double;

        // Borders of the map should always be obstacle-free. The following is done to force that always:
        if (x_cell<=0) x_cell=1;
        else if (x_cell>=no_fly_zones_[0].size()-1) x_cell=no_fly_zones_[0].size()-2;
        if (y_cell<=0) y_cell=1;
        else if (y_cell>=no_fly_zones_.size()-1) y_cell=no_fly_zones_.size()-2;

        // Finally, that cell is set to 1 in the no_fly_zones_ matrix.
        no_fly_zones_[y_cell][x_cell] = 1;
    }
}   // end method "fillCellsWithObstacles"



bool PathPlanner::visibilityCheck (const std::vector< std::pair<double,double> >& points_intersections_of_segment_with_grid) {    // This method return true if there are obstacles in the cells between points of interesection of a segment with the grid, and returns false if those cells doesn't have any obstacles.
    // "points_intersections_of_segment_with_grid" contains in a sorted way all the intersections of the segment with the grid and the beginning and the end of the segment.
    // Now the middle point between pairs of points in "points_intersections_of_segment_with_grid" are calculated. This middle points will represent points inside each cell of the segment.
    // Then, the cell of each "point_inside_cell" is calculated. If only one cell has an obstacle, break the loop and return collision_flag=1, else return 0.
    bool collision_flag = 0;
    for (int k=0; k<=points_intersections_of_segment_with_grid.size()-2; k++) {
        double point_inside_cell_x = (points_intersections_of_segment_with_grid[k].first + points_intersections_of_segment_with_grid[k+1].first)/2;   // Middle point (x axis) in double
        double point_inside_cell_y = (points_intersections_of_segment_with_grid[k].second + points_intersections_of_segment_with_grid[k+1].second)/2; // Middle point (y axis) in double

        unsigned int x_cell = (unsigned int) (point_inside_cell_x/x_cell_width_ +0.00000001);  // Cell location of "point_inside_cell" in the x axis (+ 0.00000001 to avoid numerical problems)
        unsigned int y_cell = (unsigned int) (point_inside_cell_y/y_cell_width_ +0.00000001);  // Cell location of "point_inside_cell" in the y axis (+ 0.00000001 to avoid numerical problems)

        // Saturate the grid:
        if (x_cell<0) x_cell=0;
        else if (x_cell>no_fly_zones_[0].size()-1) x_cell=no_fly_zones_[0].size()-1;
        if (y_cell<0) y_cell=0;
        else if (y_cell>no_fly_zones_.size()-1) y_cell=no_fly_zones_.size()-1;

        // Finally, that cell is checked if it has obstacle or not. If only one obstacle is found, just break the loop.
        if ( no_fly_zones_[y_cell][x_cell] == 1  ) {
            collision_flag = 1;
            break;
        }
    }
    return collision_flag;
}   // end method "visibilityCheck"



// Brief getter that returns true if the path planner is the trivial one, and false if not.
bool PathPlanner::getTrivialPathPlannerOrNot() {
    return trivial_path_planner_;
}   // end "getTrivialPathPlannerOrNot"



// Check if a test point is inside a polygon. Source: https://wrf.ecse.rpi.edu//Research/Short_Notes/pnpoly.html
bool PathPlanner::checkIfPointInsidePolygon(const std::vector<geometry_msgs::Point32>& _polygon, const geometry_msgs::Point32& _test_point) {
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



bool PathPlanner::checkIfPointInsidePolygon(const std::vector<geometry_msgs::Point32>& _polygon, const geometry_msgs::PointStamped& _test_point_stamped) {

    geometry_msgs::Point32 test_point32;
    test_point32.x = _test_point_stamped.point.x;
    test_point32.y = _test_point_stamped.point.y;
    test_point32.z = _test_point_stamped.point.z;

    return checkIfPointInsidePolygon(_polygon, test_point32);

}   // end "checkIfPointInsidePolygon" ovetloaded for geometry_msgs::PointStamped



bool PathPlanner::checkIfPointInsidePolygon(const geometry_msgs::Polygon& _polygon, const geometry_msgs::Point32& _test_point) {

    std::vector<geometry_msgs::Point32> polygon;
    for (int i=0; i<_polygon.points.size(); i++) polygon.push_back(_polygon.points[i]);

    return checkIfPointInsidePolygon(polygon, _test_point);

}   // end "checkIfPointInsidePolygon" ovetloaded for geometry_msgs::Polygon


}   // end namespace multidrone