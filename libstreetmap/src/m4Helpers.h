#pragma once

#include "globalData.h"
#include "m1.h"
#include "m1Helpers.h"
#include "m2.h"
#include "m2Helpers.h"
#include "m3Helpers.h"
#include "m4.h"
#include <atomic>

#define WORK_STOP_BUFFER_TIME 48000.0

// Similar to the node struct from m3 but without members that aren't useful here
typedef struct {
    IntersectionIdx id;
    StreetSegmentIdx prev_segment;
    double time;
    bool visited;
} smallerNode;

// Updates delivery state after arriving at one intersection
// What happens at this intersection:
// 1. Pick up every package whose pickup location is here
// 2. Drop off every package whose dropoff location is here,
//    but only if that package has already been picked up
void processIntersection(
    IntersectionIdx at,
    const std::vector<DeliveryInf>& deliveries,
    std::vector<bool>& picked_up,
    std::vector<bool>& dropped_off
);

// Returns true only when every delivery has been fully completed
// A delivery is considered completed only after its dropoff has been reached legally
bool allDone(const std::vector<bool>& dropped_off);

// Builds and returns all currently legal next intersections
// A next stop is legal if:
// 1. It is a pickup location for a delivery not yet picked up
// 2. It is a dropoff location for a delivery that has already been
//    picked up but not yet dropped off
// The returned vector stores intersections only once, even if
// multiple deliveries share the same pickup or dropoff location
std::vector<IntersectionIdx> getLegalNextStops(
    const std::vector<DeliveryInf>& deliveries,
    const std::vector<bool>& picked_up,
    const std::vector<bool>& dropped_off
);

// Chooses the best starting depot for the current route construction
// pick the depot that has the cheapest move to any legal first stop
IntersectionIdx chooseStartDepot(
    const std::vector<IntersectionIdx>& depots,
    const std::vector<IntersectionIdx>& legal_stops,
    const std::vector<std::vector<double>>& time_matrix,
    const std::map<IntersectionIdx, int>& intersection_to_matrix_index
);

// Chooses the best next legal stop from the current intersection
// pick the legal stop with the smallest travel time from current
IntersectionIdx chooseNextStop(
    IntersectionIdx current_intersection,
    const std::vector<IntersectionIdx>& legal_stops,
    const std::vector<std::vector<double>>& time_matrix,
    const std::map<IntersectionIdx, int>& intersection_to_matrix_index
);

// Builds a full greedy stop order:
// depot -> legal stop -> legal stop -> ... -> depot
// This function only builds the order of important intersections
// It does not build CourierSubPath objects yet
std::vector<IntersectionIdx> buildGreedyStopOrder(
    const std::vector<DeliveryInf>& deliveries,
    const std::vector<IntersectionIdx>& depots,
    const std::vector<std::vector<double>>& time_matrix,
    const std::map<IntersectionIdx, int>& intersection_to_matrix_index
);

std::vector<IntersectionIdx> buildGreedyStopOrderFromDepot(
    IntersectionIdx start_depot,
    const std::vector<DeliveryInf>& deliveries,
    const std::vector<IntersectionIdx>& depots,
    const std::vector<std::vector<double>>& time_matrix,
    const std::map<IntersectionIdx, int>& intersection_to_matrix_index
);

// Converts a stop order into the final CourierSubPath route
// Each consecutive pair of intersections in stop_order becomes one
// CourierSubPath, using the M3 pathfinder to get the street-segment path
std::vector<CourierSubPath> buildCourierSubPaths(
    float turn_penalty,
    const std::vector<IntersectionIdx>& stop_order
);

/// @brief For debugging, can print the matrix created by buildTravelTimeMatrix
void printMatrix(std::vector<std::vector<double>> matrix);

/// @brief Returns a vector of travel times to each target from the start intersection (in the same order as the targets vector)
/// @param turn_penalty the penalty (in seconds) of turning
/// @param start the starting intersection
/// @param targets the intersections to find a path to
std::vector<double> findTravelTimes(const double turn_penalty, const IntersectionIdx start, const std::vector<IntersectionIdx>& targets);

/**
 * @brief Returns a matrix (2d vector) of travel times between each intersection
 * @param turn_penalty the penalty (in seconds) of turning
 * @param deliveries all deliveries
 * @param depots all depots
 * @note The matrix is ordered as follows: (A = pickup, B = dropoff, D = depot)
 *           A0  ...  An  B0  ...  Bn  D0  ...  Dm
 *  A0   t(A0->A0)            ...           t(A0->Dm)
 * ...             .
 *  An                .
 *  B0        .          .                      .
 * ...        .             .                   .
 *  Bn        .                .                .
 *  D0                            .
 * ...                               .
 *  Dm   t(Dm->A0)            ...           t(Dm->Dm)
 * 
 *  matrix[from][to]
 */
std::vector<std::vector<double>> buildTravelTimeMatrix(const double turn_penalty, 
                                                       const std::vector<DeliveryInf>& deliveries, 
                                                       const std::vector<IntersectionIdx>& depots);

/// @brief A function run by a thread for building the travel time matrix
/// @param targets the targets for the matrix
/// @param available_work signals to threads which matrix rows are available to work on
/// @param matrix the matrix. It is modified by this function
void buildTravelTimeMatrixThread(const double turn_penalty, 
                                 const std::vector<IntersectionIdx>& targets,
                                 std::vector<std::atomic<bool>*>& available_work, 
                                 std::vector<std::vector<double>>& matrix);

/// @brief Returns a matrix (2d vector) of travel times between each intersection
/// @param turn_penalty the penalty (in seconds) of turning
/// @param deliveries all deliveries
/// @param depots all depots
std::vector<std::vector<double>> buildTravelTimeMatrixMultiThread(const double turn_penalty, 
                                                                  const std::vector<DeliveryInf>& deliveries, 
                                                                  const std::vector<IntersectionIdx>& depots);

/**
 * @brief Returns a map that can be used to convert an intersection id to an index to use with the matrix
 * @param deliveries all deliveries
 * @param depots all depots
 */
std::map<IntersectionIdx, int> buildMatrixSearchMap(const std::vector<DeliveryInf>& deliveries,
                                                    const std::vector<IntersectionIdx>& depots);

/// @brief Uses the travel time matrix and its map to get the travel time between two intersections
/// @param intersections The intersections {from, to}
double getTimeBetweenIntersections(const std::pair<IntersectionIdx, IntersectionIdx>& intersections,
                                   const std::vector<std::vector<double>>& matrix, 
                                   const std::map<IntersectionIdx, int>& search_map);

// Returns the total matrix travel time of one stop order.
// Used to compare multi-start candidates and keep the best one.
double computeStopOrderTravelTime(
    const std::vector<IntersectionIdx>& stop_order,
    const std::vector<std::vector<double>>& matrix,
    const std::map<IntersectionIdx, int>& search_map
);

void optimizationThread(std::vector<IntersectionIdx>& route,
                        const std::vector<DeliveryInf>& deliveries, 
                        const std::vector<std::vector<double>>& time_matrix, 
                        const std::map<IntersectionIdx, int>& search_map,
                        const double allotted_time);

/// @brief Finds an optimal heuristic order of intersections using the 2-opt
/// TIMES ARE IN MILISECONDS
std::vector<IntersectionIdx> optimizeVia2_Opt (std::vector<IntersectionIdx>& route, 
                                               const std::vector<DeliveryInf>& deliveries, 
                                               const std::vector<std::vector<double>>& time_matrix, 
                                               const std::map<IntersectionIdx, int>& search_map,
                                               double allotted_time);
/// @brief Finds an optimal heuristic order of intersection using the swap technique
std::vector<IntersectionIdx> optimizeViaSwap (std::vector<IntersectionIdx>& route, 
                                              const std::vector<DeliveryInf>& deliveries, 
                                              const std::vector<std::vector<double>>& time_matrix, 
                                              const std::map<IntersectionIdx, int>& search_map,
                                              double allotted_time);

/// @brief Creates a viable starting route using randomize greedy approach
/// @return returns the K best route found
std::vector<IntersectionIdx> buildRandomizedGreedyStopOrderFromDepot(
    IntersectionIdx start_depot,
    const std::vector<DeliveryInf>& deliveries,
    const std::vector<IntersectionIdx>& depots,
    const std::vector<std::vector<double>>& time_matrix,
    const std::map<IntersectionIdx, int>& intersection_to_matrix_index,
    int K   // randomness factor (recommend 3–6)
);

/// @brief selects a random intersection within delivery points as next stop in route
IntersectionIdx chooseNextStopRandomized(
    IntersectionIdx current_intersection,
    const std::vector<IntersectionIdx>& legal_stops,
    const std::vector<std::vector<double>>& time_matrix,
    const std::map<IntersectionIdx, int>& intersection_to_matrix_index,
    int K // number of best candidates to consider
);
//////

/// @brief Finds an optimal heuristic order of intersection using the or-opt technique
std::vector<IntersectionIdx> optimizeViaRelocate(
    std::vector<IntersectionIdx>& route,
    const std::vector<DeliveryInf>& deliveries,
    const std::vector<std::vector<double>>& matrix,
    const std::map<IntersectionIdx, int>& search_map,
    double allotted_time
);
/////