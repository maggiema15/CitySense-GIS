#include "globalData.h"
#include "m1.h"
#include "m1Helpers.h"
#include "m2.h"
#include "m2Helpers.h"
#include "m3Helpers.h"
#include <queue>
#include "ezgl/application.hpp"
#include "ezgl/graphics.hpp"

std::vector<std::string> build_path_directions(const std::vector<StreetSegmentIdx>& path) {
    std::vector<std::string> directions;

    // No path means no directions
    if (path.empty()) {
        return directions;
    }

    std::vector<StreetSegmentIdx> forward_path = path;

    // Start with the first street in the route
    StreetSegmentIdx first_seg = forward_path[0];
    StreetIdx current_street_id = getStreetSegmentInfo(first_seg).streetID;
    std::string current_street_name = getStreetName(current_street_id);

    if (current_street_name.empty() || current_street_name == "<unknown>") {
        current_street_name = "Unnamed Street";
    }

    // First instruction: begin the route on the starting street
    directions.push_back("↑ Go straight on " + current_street_name);

    // Walk through the rest of the path and add a new instruction
    // whenever the street name changes
    for (size_t i = 1; i < forward_path.size(); i++) {
        StreetSegmentIdx prev_seg = forward_path[i - 1];
        StreetSegmentIdx curr_seg = forward_path[i];

        StreetIdx prev_street_id = getStreetSegmentInfo(prev_seg).streetID;
        StreetIdx curr_street_id = getStreetSegmentInfo(curr_seg).streetID;

        std::string prev_street_name = getStreetName(prev_street_id);
        std::string next_street_name = getStreetName(curr_street_id);

        if (next_street_name.empty() || next_street_name == "<unknown>") {
            next_street_name = "Unnamed Street";
        }

        // If still on the same street, do not create a new direction line
        if (prev_street_id == curr_street_id) {
            continue;
        }

        // Use the signed turn angle between consecutive segments
        // Positive angle -> left turn
        // Negative angle -> right turn
        double turn_angle = findStreetSegmentTurnAngle(curr_seg, prev_seg);

        // Small angle means keep going mostly straight
        if (std::abs(turn_angle) < 0.35) {
            directions.push_back("↑ Go straight on " + next_street_name);
        }
        else if (turn_angle > 0) {
            directions.push_back("← Turn left onto " + next_street_name);
        }
        else {
            directions.push_back("→ Turn right onto " + next_street_name);
        }
    }

    return directions;
}

std::vector<StreetSegmentIdx> findDrivePath(const double turn_penalty, const std::pair<IntersectionIdx, IntersectionIdx> intersect_ids, std::vector<IntersectionIdx> force_queue) {
    // Create nodes map
    std::unordered_map<IntersectionIdx, node> nodes;
    // Create priority queue
    std::priority_queue<std::pair<IntersectionIdx, double>, std::vector<std::pair<IntersectionIdx, double>>, Compare> to_visit;
    // Handle force queue for when we want the search to begin with pre loaded data
    for (auto intersection : force_queue) {
        node temp = {intersection, {NULL, NULL}, 0.0, 0.0, false};
        nodes.emplace(intersection, temp);
        to_visit.push({intersection, 0.0});
    }
    to_visit.push({intersect_ids.first, 0.0});
    // Set up node map
    node temp1 = {intersect_ids.first, {NULL, NULL}, 0.0, 0.0, false};
    nodes.emplace(intersect_ids.first, temp1);
    node* current = &(nodes.find(intersect_ids.first)->second);
    // For determining unreasonable searches
    double max_cost = 600.0 + findDistanceBetweenTwoPoints({getIntersectionPosition(intersect_ids.first), 
                                                            getIntersectionPosition(intersect_ids.second)});

    // Main loop
    while (current->id != intersect_ids.second && to_visit.size() > 0) {
        current = &(nodes.find(to_visit.top().first)->second);
        to_visit.pop();
        if (current->visited) { continue; }
        // Stop condition for unreasonable searches
        if (current->cost > max_cost) { return {}; }

        std::vector<StreetSegmentIdx> segments = findStreetSegmentsOfIntersection(current->id);
        std::vector<std::pair<StreetSegmentIdx, IntersectionIdx>> branches;
        // Create vector of available branches
        for (auto &segment : segments) {
            if (current->id == getStreetSegmentInfo(segment).to && !getStreetSegmentInfo(segment).oneWay) {
                branches.push_back({segment, getStreetSegmentInfo(segment).from});
            } else if (current->id == getStreetSegmentInfo(segment).from) {
                branches.push_back({segment, getStreetSegmentInfo(segment).to});
            }
        }

        // Check branch
        for (auto &branch : branches) {
            // Is this a turn?
            double turn_cost = 0.0;
            if (getStreetSegmentInfo(current->prev.first).streetID != getStreetSegmentInfo(branch.first).streetID) {
                turn_cost = turn_penalty;
            }
            double time = findDriveTime(turn_cost, current->time, findStreetSegmentTravelTime(branch.first));
            double cost = findDriveCost(time, branch.second, intersect_ids.second);
            auto intersection = nodes.find(branch.second);
            // Intersection never viewed -- create entry
            if (intersection == nodes.end()) {
                node temp2 = {branch.second, {branch.first, current->id}, time, cost, false};
                nodes.emplace(branch.second, temp2);
                to_visit.push({branch.second, cost});
            // Intersection has been viewed and current route is better than existing
            } else if (intersection->second.cost > cost) {
                intersection->second.time = time;
                intersection->second.cost = cost;
                intersection->second.prev = {branch.first, current->id};
                to_visit.push({branch.second, intersection->second.cost});
            }
        }
        current->visited = true;
    }
    std::vector<StreetSegmentIdx> path;
    // No path
    if (to_visit.size() == 0) { std::cout << "findDrivePath: no path found" << std::endl; return path; }

    // Construct route
    while (current->time != 0.0) {
        // DEBUG - std::cout << getIntersectionName(current->id) << std::endl;
        path.push_back(current->prev.first);
        current = &(nodes.find(current->prev.second)->second);
    }
    // DEBUG - std::cout << "Search iterations: " << iterations << std::endl;
    std::reverse(path.begin(), path.end());
    return path;
}   

double findDriveTime(const double turn_cost, double current_time, double segment_travel_time) {
    return current_time + segment_travel_time + turn_cost;
}

double findDriveCost(double time, IntersectionIdx observing, IntersectionIdx dst) {
    return time + (findDistanceBetweenTwoPoints({getIntersectionPosition(observing), getIntersectionPosition(dst)}) / ESTIMATE_SPEED_MAX);
}

std::vector<StreetSegmentIdx> findWalkPath(const double turn_penalty, const std::pair<IntersectionIdx, IntersectionIdx> intersect_ids, 
                                           const double walk_speed) {
    // Create nodes map
    std::unordered_map<IntersectionIdx, node> nodes;
    // Create priority queu
    std::priority_queue<std::pair<IntersectionIdx, double>, std::vector<std::pair<IntersectionIdx, double>>, Compare> to_visit;
    to_visit.push({intersect_ids.first, 0});
    // Set up node map for observed intersections
    node temp1 = {intersect_ids.first, {NULL, NULL}, 0.0, 0.0, false};
    nodes.emplace(intersect_ids.first, temp1);
    node* current = &(nodes.begin()->second);   

    // Main loop
    while (current->id != intersect_ids.second) {
        current = &(nodes.find(to_visit.top().first)->second);
        to_visit.pop();
        if (current->visited) { continue; }
        std::vector<StreetSegmentIdx> segments = findStreetSegmentsOfIntersection(current->id);
        std::vector<std::pair<StreetSegmentIdx, IntersectionIdx>> branches;

        // Create vector of available branches
        for (auto &segment : segments) {
            if (current->id == getStreetSegmentInfo(segment).to) {
                branches.push_back({segment, getStreetSegmentInfo(segment).from});
            } else if (current->id == getStreetSegmentInfo(segment).from) {
                branches.push_back({segment, getStreetSegmentInfo(segment).to});
            }
        }

        // Check branch
        for (auto &branch : branches) {
            // Is this a turn?
            double turn_cost = 0.0;
            if (getStreetSegmentInfo(current->prev.first).streetID != getStreetSegmentInfo(branch.first).streetID) {
                turn_cost = turn_penalty;
            }
            double time = findWalkTime(turn_cost, current->time, findStreetSegmentLength(branch.first), walk_speed);
            double cost = findWalkCost(time, walk_speed, branch.second, intersect_ids.second);
            auto intersection = nodes.find(branch.second);
            // Intersection never viewed -- create entry
            if (intersection == nodes.end()) {
                node temp2 = {branch.second, {branch.first, current->id}, time, cost, false};
                nodes.emplace(branch.second, temp2);
                to_visit.push({branch.second, time});
            // Intersection has been viewed and current route is better than existing
            } else if (intersection->second.cost > cost) {
                intersection->second.time = time;
                intersection->second.cost = cost;
                intersection->second.prev = {branch.first, current->id};
                to_visit.push({branch.second, intersection->second.cost});
            }
        }
        current->visited = true;
    }
    std::vector<StreetSegmentIdx> path;
    // Construct route
    while (current->id != intersect_ids.first) {
        // DEBUG - std::cout << getIntersectionName(current->id) << std::endl;
        path.push_back(current->prev.first);
        current = &(nodes.find(current->prev.second)->second);
    }
    // DEBUG - std::cout << "Search iterations: " << iterations << std::endl;
    std::reverse(path.begin(), path.end());
    return path;
}


std::vector<IntersectionIdx> buildWalkingRadius(const double turn_penalty, const IntersectionIdx start, 
                                                const double walk_speed, const double walk_time_limit) {
    // Create nodes map
    std::unordered_map<IntersectionIdx, node> nodes;
    // Create priority queue -- note: sorted by time here
    std::priority_queue<std::pair<IntersectionIdx, double>, std::vector<std::pair<IntersectionIdx, double>>, Compare> to_visit;
    to_visit.push({start, 0.0});
    // Set up node map for observed intersections
    node temp1 = {start, {NULL, NULL}, -turn_penalty, 0.0, false};
    nodes.emplace(start, temp1);
    node* current = &(nodes.begin()->second);   

    while (current->time <= walk_time_limit) {
        current = &(nodes.find(to_visit.top().first)->second);
        to_visit.pop();
        if (current->visited) { continue; }
        std::vector<StreetSegmentIdx> segments = findStreetSegmentsOfIntersection(current->id);
        std::vector<std::pair<StreetSegmentIdx, IntersectionIdx>> branches;

        // Create vector of available branches
        for (auto &segment : segments) {
            if (current->id == getStreetSegmentInfo(segment).to) {
                branches.push_back({segment, getStreetSegmentInfo(segment).from});
            } else if (current->id == getStreetSegmentInfo(segment).from) {
                branches.push_back({segment, getStreetSegmentInfo(segment).to});
            }
        }

        // Check branch
        for (auto &branch : branches) {
            // Is this a turn?
            double turn_cost = 0.0;
            if (getStreetSegmentInfo(current->prev.first).streetID != getStreetSegmentInfo(branch.first).streetID) {
                turn_cost = turn_penalty;
            }
            double time = findWalkTime(turn_cost, current->time, findStreetSegmentLength(branch.first), walk_speed);
            auto intersection = nodes.find(branch.second);
            // Intersection never viewed -- create entry
            if (intersection == nodes.end()) {
                node temp2 = {branch.second, {branch.first, current->id}, time, 0.0, false};
                nodes.emplace(branch.second, temp2);
                to_visit.push({branch.second, time});
            // Intersection has been viewed and current route is better than existing
            } else if (intersection->second.time > time) {
                intersection->second.time = time;
                intersection->second.prev = {branch.first, current->id};
                to_visit.push({branch.second, intersection->second.time});
            }
        }
        current->visited = true;
    }
    std::vector<IntersectionIdx> walkable_intersections;
    for (auto intersection : nodes) {
        if (intersection.second.time <= walk_time_limit) {
            walkable_intersections.push_back(intersection.first);
        }
    }
    return walkable_intersections;
}

double findWalkTime(const double turn_cost, double current_time, double segment_length, double walk_speed) {
    return current_time + (segment_length / walk_speed) + turn_cost;
}

double findWalkCost(double time, double walk_speed, IntersectionIdx observing, IntersectionIdx dst) {
    return time + (findDistanceBetweenTwoPoints({getIntersectionPosition(observing), getIntersectionPosition(dst)}) / walk_speed);
}

int get_max_intersections_clicked(ezgl::application *app) {
    GtkToggleButton *find_path_from_clicked_intersections = GTK_TOGGLE_BUTTON(app->get_object("EnablePathOfTwoIntersections"));
    if (gtk_toggle_button_get_active(find_path_from_clicked_intersections)) {
        //std::cout << "toggled, therefore max = 2 " <<std::endl;
        return 2;
    }
    else {
        //std::cout << "NOT toggled, therefore max = 1 " <<std::endl;
        return 1;
    }
}

bool get_drive_only_clicked(ezgl::application *app) {
    GtkToggleButton *drive_only_toggle = GTK_TOGGLE_BUTTON(app->get_object("EnableDriveOnly"));
    if (gtk_toggle_button_get_active(drive_only_toggle)) {
        std::cout << "drive-only toggled" << std::endl;
        return true;
    }
    
    std::cout << "drive + walk enabled" << std::endl;
    return false;
}



