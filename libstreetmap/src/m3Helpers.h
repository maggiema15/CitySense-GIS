#pragma once

#include "globalData.h"
#include "m1.h"
#include "m1Helpers.h"
#include "m2.h"
#include "m2Helpers.h"
#include "ezgl/application.hpp"
#include "ezgl/graphics.hpp"

#define ESTIMATE_SPEED_MAX 25.0 // For A* heuristic calculation

//stores the walking speeds available, and stores the option chosen by the user
//walking speeds are in m/s
//the average speeds were obtained from https://www.researchgate.net/figure/Mean-walking-speed-m-s-The-dashed-lines-represent-the-trend-for-mean-speed-by-pace_fig4_344609180
const double kwalk_speed = 0.8;  
const double kjog_speed =  1.4;
const double krun_speed =  1.7;


std::vector<std::string> build_path_directions(const std::vector<StreetSegmentIdx>& path);

// A node represents an intersection that the algorithm has viewed at least once during the current search
typedef struct {
    IntersectionIdx id;
    std::pair<StreetSegmentIdx, IntersectionIdx> prev;
    double time;     // Amount of time to travel to a node
    double cost;     // Computed cost
    bool visited;
} node;

// To allow the queue within the pathfinding functions to compare its contents for sorting
struct Compare {
    bool operator()(const std::pair<IntersectionIdx, double>& a, const std::pair<IntersectionIdx, double>& b) const {
        return a.second > b.second;
    }
};

// Uses an implmentation of A* to find the optimal driving path between two intersections
std::vector<StreetSegmentIdx> findDrivePath(const double turn_penalty, const std::pair<IntersectionIdx, IntersectionIdx> intersect_ids, std::vector<IntersectionIdx> force_queue);

// Returns the time required to drive to the passed intersection
double findDriveTime(const double turn_cost, double current_time, double segment_travel_time);

// Returns the "cost" for driving to the passed intersection, taking into account its distance from the target destination
double findDriveCost(double time, IntersectionIdx observing, IntersectionIdx dst);

// Uses an implmentation of A* to find the optimal walking path between two intersections
std::vector<StreetSegmentIdx> findWalkPath(const double turn_penalty, const std::pair<IntersectionIdx, IntersectionIdx> intersect_ids, 
                                           const double walk_speed);

// Uses an implementation of Dijkstra's algorithm to create a vector of intersections reachable within the given time limit
std::vector<IntersectionIdx> buildWalkingRadius(const double turn_penalty, const IntersectionIdx start, 
                                                const double walk_speed, const double walk_time_limit);

// Returns the time required to walk to the passed intersection
double findWalkTime(const double turn_cost, double current_time, double segment_length, double walk_speed);

// Returns the "cost" for walking to the passed intersetion, taking into account its distance from the target destination
double findWalkCost(double time, double walk_speed, IntersectionIdx observing, IntersectionIdx dst);

int get_max_intersections_clicked(ezgl::application *app);

bool get_drive_only_clicked(ezgl::application *app);



