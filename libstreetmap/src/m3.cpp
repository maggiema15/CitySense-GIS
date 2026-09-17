/*
This file includes the implementations of the m3.h file provided by the course 
Team cd-47:
    Arshiya Mostafavisabet
    Maggie Ma
    William Forbes Flag
*/

//  header inclusions   //
#include "StreetsDatabaseAPI.h"
#include "globalData.h"
#include "m1.h"
#include "m2.h"
#include "m3.h"
#include "m3Helpers.h"
extern std::map<IntersectionIdx, node> nodes;

// function implementations

double computePathTravelTime(const double turn_penalty, const std::vector<StreetSegmentIdx>& path) {
    //(void) turn_penalty;
    //(void) path;

    // Empty path
    if (path.empty()) {
        return 0.0;
    }

    double total_travel_time = 0.0;

    for (size_t i = 0; i < path.size(); i++) {
        StreetSegmentIdx current_segment = path[i];

        // Add the precomputed driving time for this segment
        total_travel_time += findStreetSegmentTravelTime(current_segment);

        // Add turn penalty if this segment is on a different street
        // from the previous segment
        if (i > 0) {
            StreetSegmentIdx previous_segment = path[i - 1];

            StreetIdx previous_street_id = getStreetSegmentInfo(previous_segment).streetID;
            StreetIdx current_street_id = getStreetSegmentInfo(current_segment).streetID;

            if (previous_street_id != current_street_id) {
                total_travel_time += turn_penalty;
            }
        }
    }

    return total_travel_time;
}

std::vector<StreetSegmentIdx> findPathBetweenIntersections(const double turn_penalty, const std::pair<IntersectionIdx, IntersectionIdx> intersection_ids) {
    std::vector<StreetSegmentIdx> pathBetweenIntersections = findDrivePath(turn_penalty, intersection_ids, {});

    return pathBetweenIntersections;
}

double computePathWalkingTime(const std::vector<StreetSegmentIdx>& path,
                            const double walking_speed, 
                            const double turn_penalty) {

    //(void) path;
    //(void) walking_speed;
    (void) turn_penalty;

    if (path.empty()) {
        return 0.0;
    }

    // Invalid walking speeds
    if (walking_speed <= 0.0) {
        return 0.0;
    }

    double total_walking_time = 0.0;

    for (size_t i = 0; i < path.size(); i++) {
        StreetSegmentIdx current_segment = path[i];

        // Walking time along this segment
        total_walking_time += findStreetSegmentLength(current_segment) / walking_speed;

        // turn_penalty also applies while walking
        if (i > 0) {
            StreetSegmentIdx previous_segment = path[i - 1];

            StreetIdx previous_street_id = getStreetSegmentInfo(previous_segment).streetID;
            StreetIdx current_street_id = getStreetSegmentInfo(current_segment).streetID;

            if (previous_street_id != current_street_id) {
                total_walking_time += turn_penalty;
            }
        }
    }

    return total_walking_time;
}

std::pair<std::vector<StreetSegmentIdx>, std::vector<StreetSegmentIdx>> findPathWithWalkToPickUp(
    const IntersectionIdx start_intersection, 
    const IntersectionIdx end_intersection, 
    const double turn_penalty, 
    const double walking_speed,
    const double walking_time_limit
) {
    /* 
        Overview: Find walk path first that minimizes remaining distance, then use drive pathing from there
    
    std::vector<StreetSegmentIdx> walk_path = findWalkPath(turn_penalty, {start_intersection, end_intersection}, 
                                                           walking_speed, walking_time_limit);

    IntersectionIdx drive_start = start_intersection;
    if (!walk_path.empty()) {
        StreetSegmentInfo last_segment_info = getStreetSegmentInfo(walk_path.back());
        if (walk_path.size() == 1) {
            if (last_segment_info.from == start_intersection) { drive_start = last_segment_info.to; }
            else { drive_start = last_segment_info.from; }
        } else if (last_segment_info.from == getStreetSegmentInfo(*(walk_path.rbegin() + 1)).from ||
                   last_segment_info.from == getStreetSegmentInfo(*(walk_path.rbegin() + 1)).to) {
            drive_start = last_segment_info.to;
        } else { drive_start = last_segment_info.from; }
    }

    std::vector<StreetSegmentIdx> drive_path = findDrivePath(turn_penalty, {drive_start, end_intersection});

    return {walk_path, drive_path};
    
    std::vector<StreetSegmentIdx> drive_path = findDrivePath(turn_penalty, {start_intersection, end_intersection});
    std::vector<StreetSegmentIdx> walk_path;
    
    IntersectionIdx last_end = start_intersection;
    int i = 0;
    for (; i < drive_path.size(); i++) {
        StreetSegmentInfo segment_info = getStreetSegmentInfo(drive_path[i]);
        IntersectionIdx to_test;
        if (segment_info.to == last_end) {
            to_test = segment_info.from;
        } else (to_test = segment_info.to);

        std::vector<StreetSegmentIdx> test_path = findWalkPath(turn_penalty, {start_intersection, to_test}, walking_speed, walking_time_limit);
        if (test_path.size() == 0) { break; }
        last_end = to_test;
        walk_path = test_path;
    }
    drive_path = findDrivePath(turn_penalty, {last_end, end_intersection});
    return {walk_path, drive_path};
    */

    std::vector<IntersectionIdx> walkable_intersections = buildWalkingRadius(turn_penalty, start_intersection, 
                                                                             walking_speed, walking_time_limit);
    // Search with all walkable intersections set to have a cost of 0
    std::vector<StreetSegmentIdx> drive_path = findDrivePath(turn_penalty, {start_intersection, end_intersection}, walkable_intersections);
    std::vector<StreetSegmentIdx> walk_path;
    if (walking_time_limit > 0.0) {
        IntersectionIdx walk_end;
        // Find end intersection of walk_path (start intersection of drive_path)
        if (drive_path.size() > 1) {
            if (getStreetSegmentInfo(drive_path[0]).from == getStreetSegmentInfo(drive_path[1]).from
             || getStreetSegmentInfo(drive_path[0]).from == getStreetSegmentInfo(drive_path[1]).to) {
                walk_end = getStreetSegmentInfo(drive_path[0]).to;
            } else { walk_end = getStreetSegmentInfo(drive_path[0]).from; }
        } else if (drive_path.size() == 1) {
            if (getStreetSegmentInfo(drive_path[0]).from == end_intersection) {
                walk_end = getStreetSegmentInfo(drive_path[0]).to;
            } else { walk_end = getStreetSegmentInfo(drive_path[0]).from; }
        } else { walk_end = end_intersection; }
        walk_path = findWalkPath(turn_penalty, {start_intersection, walk_end}, walking_speed);
    }
    return {walk_path, drive_path};
}