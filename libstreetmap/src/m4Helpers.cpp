#include "m3Helpers.h"
#include "m4Helpers.h"

#include <limits>
#include <set>
#include <queue>
#include <chrono>
#include <thread>
#include <atomic>

void processIntersection(
    IntersectionIdx current_intersection,
    const std::vector<DeliveryInf>& deliveries,
    std::vector<bool>& picked_up,
    std::vector<bool>& dropped_off) {
    // First pass:
    // Pick up every package whose pickup location is this intersection
    // Do not change dropped_off here
    for (size_t i = 0; i < deliveries.size(); i++) {
        if (!picked_up[i] && deliveries[i].pickUp == current_intersection) {
            picked_up[i] = true;
        }
    }
    // Second pass:
    // Drop off every package whose dropoff location is this intersection,
    // but only if it has already been picked up and not already dropped off
    for (size_t i = 0; i < deliveries.size(); i++) {
        if (picked_up[i] && !dropped_off[i] &&
            deliveries[i].dropOff == current_intersection) {
            dropped_off[i] = true;
        }
    }
}

bool allDone(const std::vector<bool>& dropped_off) {
    // If any delivery has not been dropped off yet, then the full courier route is not finished
    for (size_t i = 0; i < dropped_off.size(); i++) {
        if (dropped_off[i] == false) {
            return false;
        }
    }
    return true;
}

std::vector<IntersectionIdx> getLegalNextStops(
    const std::vector<DeliveryInf>& deliveries,
    const std::vector<bool>& picked_up,
    const std::vector<bool>& dropped_off) {
    
    std::vector<IntersectionIdx> legal_stops;

    for (size_t i = 0; i < deliveries.size(); i++) {
        // Case 1:
        // Pickup has not happened yet, so its pickup intersection is a legal next stop
        if (picked_up[i] == false) {
            IntersectionIdx pickup_intersection = deliveries[i].pickUp;

            bool already_added = false;
            for (size_t j = 0; j < legal_stops.size(); j++) {
                if (legal_stops[j] == pickup_intersection) {
                    already_added = true;
                    break;
                }
            }
            if (already_added == false) {
                legal_stops.push_back(pickup_intersection);
            }
        }
        // Case 2:
        // Pickup already happened, but dropoff has not happened yet,
        // so its dropoff intersection is a legal next stop
        if (picked_up[i] == true && dropped_off[i] == false) {
            IntersectionIdx dropoff_intersection = deliveries[i].dropOff;

            bool already_added = false;
            for (size_t j = 0; j < legal_stops.size(); j++) {
                if (legal_stops[j] == dropoff_intersection) {
                    already_added = true;
                    break;
                }
            }
            if (already_added == false) {
                legal_stops.push_back(dropoff_intersection);
            }
        }
    }
    return legal_stops;
}

IntersectionIdx chooseStartDepot(
    const std::vector<IntersectionIdx>& depots,
    const std::vector<IntersectionIdx>& legal_stops,
    const std::vector<std::vector<double>>& time_matrix,
    const std::map<IntersectionIdx, int>& intersection_to_matrix_index) {
    
    // Start with the first depot as the default answer.
    IntersectionIdx best_depot = depots[0];
    double best_time = std::numeric_limits<double>::infinity();

    // Try every depot against every legal first stop.
    for (size_t i = 0; i < depots.size(); i++) {
        IntersectionIdx depot = depots[i];

        int depot_index = intersection_to_matrix_index.at(depot);

        for (size_t j = 0; j < legal_stops.size(); j++) {
            IntersectionIdx stop = legal_stops[j];
            int stop_index = intersection_to_matrix_index.at(stop);

            double current_time = time_matrix[depot_index][stop_index];

            if (current_time < best_time) {
                best_time = current_time;
                best_depot = depot;
            }
        }
    }
    return best_depot;
}

IntersectionIdx chooseNextStop(
    IntersectionIdx current_intersection,
    const std::vector<IntersectionIdx>& legal_stops,
    const std::vector<std::vector<double>>& time_matrix,
    const std::map<IntersectionIdx, int>& intersection_to_matrix_index) {
    
    // Use the first legal stop as default
    IntersectionIdx best_stop = legal_stops[0];
    double best_time = std::numeric_limits<double>::infinity();

    int current_index = intersection_to_matrix_index.at(current_intersection);

    // Compare all legal next stops and keep the cheapest one
    for (size_t i = 0; i < legal_stops.size(); i++) {
        IntersectionIdx candidate_stop = legal_stops[i];
        int candidate_index = intersection_to_matrix_index.at(candidate_stop);

        double current_time = time_matrix[current_index][candidate_index];
        if (current_time < best_time) {
            best_time = current_time;
            best_stop = candidate_stop;
        }
    }
    return best_stop;
}


IntersectionIdx chooseNextStopRandomized(
    IntersectionIdx current_intersection,
    const std::vector<IntersectionIdx>& legal_stops,
    const std::vector<std::vector<double>>& time_matrix,
    const std::map<IntersectionIdx, int>& intersection_to_matrix_index,
    int K 
) {
    //get the index of intersection in the time_matrix
    int current_index = intersection_to_matrix_index.at(current_intersection);

    //create a vector of candidate stop points
    //each hold travel_time and their id
    std::vector<std::pair<double, IntersectionIdx>> candidates;

    //cache the candidate vector infos
    for (size_t stop_index = 0; stop_index < legal_stops.size(); stop_index++) {
        IntersectionIdx stop = legal_stops[stop_index];
        int idx = intersection_to_matrix_index.at(stop);

        double time = time_matrix[current_index][idx];
        candidates.push_back({time, stop});
    }

    // sort the candidates based on the travel times from current intersection
    std::sort(candidates.begin(), candidates.end());

    // to save time, limit the number of candidates to K
    int limit = std::min(K, (int)candidates.size());

    // randomly pick between any of the candidates as the random next stop
    int chosen = rand() % limit;

    return candidates[chosen].second;
}


std::vector<IntersectionIdx> buildGreedyStopOrder(
    const std::vector<DeliveryInf>& deliveries,
    const std::vector<IntersectionIdx>& depots,
    const std::vector<std::vector<double>>& time_matrix,
    const std::map<IntersectionIdx, int>& intersection_to_matrix_index) {
        
    std::vector<IntersectionIdx> stop_order;

    // Track delivery state
    std::vector<bool> picked_up(deliveries.size(), false);
    std::vector<bool> dropped_off(deliveries.size(), false);

    // At the beginning, only pickups are legal
    std::vector<IntersectionIdx> legal_stops = getLegalNextStops(deliveries, picked_up, dropped_off);

    // If there is somehow no legal first stop, fail early
    if (legal_stops.empty() == true) {
        return {};
    }

    // Choose the depot that gives the best first move
    IntersectionIdx start_depot = chooseStartDepot(
        depots,
        legal_stops,
        time_matrix,
        intersection_to_matrix_index
    );

    stop_order.push_back(start_depot);

    IntersectionIdx current_intersection = start_depot;

    // Keep going until every delivery has been dropped off
    while (allDone(dropped_off) == false) {
        legal_stops = getLegalNextStops(deliveries, picked_up, dropped_off);

        // If no legal stop exists before all deliveries are done,
        // then this route construction fails
        if (legal_stops.empty() == true) {
            return {};
        }

        IntersectionIdx next_stop = chooseNextStop(
            current_intersection,
            legal_stops,
            time_matrix,
            intersection_to_matrix_index
        );

        stop_order.push_back(next_stop);
        current_intersection = next_stop;

        // Update pickup/dropoff state after arriving here.
        processIntersection(current_intersection, deliveries, picked_up, dropped_off);
    }

    // Choose the best depot to end at
    IntersectionIdx best_end_depot = depots[0];
    double best_end_time = std::numeric_limits<double>::infinity();

    int current_index = intersection_to_matrix_index.at(current_intersection);

    for (size_t i = 0; i < depots.size(); i++) {
        IntersectionIdx depot = depots[i];
        int depot_index = intersection_to_matrix_index.at(depot);

        double current_time = time_matrix[current_index][depot_index];

        if (current_time < best_end_time) {
            best_end_time = current_time;
            best_end_depot = depot;
        }
    }

    stop_order.push_back(best_end_depot);

    return stop_order;
}

std::vector<IntersectionIdx> buildGreedyStopOrderFromDepot(
    IntersectionIdx start_depot,
    const std::vector<DeliveryInf>& deliveries,
    const std::vector<IntersectionIdx>& depots,
    const std::vector<std::vector<double>>& time_matrix,
    const std::map<IntersectionIdx, int>& intersection_to_matrix_index) {
        
    std::vector<IntersectionIdx> stop_order;

    // Track delivery state for this greedy run
    std::vector<bool> picked_up(deliveries.size(), false);
    std::vector<bool> dropped_off(deliveries.size(), false);

    // At the beginning, only pickups are legal
    std::vector<IntersectionIdx> legal_stops =
        getLegalNextStops(deliveries, picked_up, dropped_off);

    // Fail early if there is no legal first stop
    if (legal_stops.empty() == true) {
        return {};
    }

    // Force this greedy run to start from the depot passed in
    stop_order.push_back(start_depot);
    IntersectionIdx current_intersection = start_depot;

    // Keep choosing the cheapest legal next stop until all deliveries are done
    while (allDone(dropped_off) == false) {
        legal_stops = getLegalNextStops(deliveries, picked_up, dropped_off);

        if (legal_stops.empty() == true) {
            return {};
        }

        IntersectionIdx next_stop = chooseNextStop(
            current_intersection,
            legal_stops,
            time_matrix,
            intersection_to_matrix_index
        );
        

        stop_order.push_back(next_stop);
        current_intersection = next_stop;

        // Update delivery state after arriving at this stop
        processIntersection(current_intersection, deliveries, picked_up, dropped_off);
    }

    // End at the cheapest depot from the last delivery-related stop
    IntersectionIdx best_end_depot = depots[0];
    double best_end_time = std::numeric_limits<double>::infinity();

    int current_index = intersection_to_matrix_index.at(current_intersection);

    for (size_t i = 0; i < depots.size(); i++) {
        IntersectionIdx depot = depots[i];
        int depot_index = intersection_to_matrix_index.at(depot);

        double current_time = time_matrix[current_index][depot_index];

        if (current_time < best_end_time) {
            best_end_time = current_time;
            best_end_depot = depot;
        }
    }

    stop_order.push_back(best_end_depot);

    return stop_order;
}

std::vector<CourierSubPath> buildCourierSubPaths(
    float turn_penalty,
    const std::vector<IntersectionIdx>& stop_order) {

    std::vector<CourierSubPath> courier_route;

    // Need at least two intersections to form one subpath
    if (stop_order.size() < 2) {
        std::cout << "Error: stop order is too small." << std::endl;
        return {};
    }

    for (size_t i = 0; i + 1 < stop_order.size(); i++) {
        IntersectionIdx start_intersection = stop_order[i];
        IntersectionIdx end_intersection = stop_order[i + 1];

        std::vector<StreetSegmentIdx> path = findDrivePath(turn_penalty, {start_intersection, end_intersection}, {});

        // If start and end are different but no path exists, fail
        if (start_intersection != end_intersection && path.empty() == true) {
            std::cout << "Error: an invalid path was found during courier subpath building: (" 
                      << getIntersectionPosition(start_intersection) << " with connections ";
            for (auto &it : findAdjacentIntersections(start_intersection)) {
                std::cout << it << " ";
            }
            std::cout << " -> " << getIntersectionPosition(end_intersection) << " with connections ";
            for (auto &it : findAdjacentIntersections(end_intersection)) {
                std::cout << it << " ";
            }
            std::cout << "). " << std::endl << std::endl;
            return {};
        }

        CourierSubPath subpath;
        subpath.intersections = {start_intersection, end_intersection};
        subpath.subpath = path;

        courier_route.push_back(subpath);
    }
    return courier_route;
}

// For debugging, can print the matrix created by buildTravelTimeMatrix
void printMatrix(std::vector<std::vector<double>> matrix) {
    std::cout << "Travel Time Matrix:" << std::endl;
    for (int i = 0; i < matrix.size(); i++) {
        for (int j = 0; j < matrix.size(); j++) {
            std::cout << matrix[i][j] << " ";
        }
        std::cout << std::endl;
    }
}

// Returns a vector of travel times to each target from the start intersection (in the same order as the targets vector)
std::vector<double> findTravelTimes(const double turn_penalty, const IntersectionIdx start, const std::vector<IntersectionIdx>& targets) {
    // Create nodes map
    std::unordered_map<IntersectionIdx, smallerNode> nodes;
    // Create priority queue
    std::priority_queue<std::pair<IntersectionIdx, double>, std::vector<std::pair<IntersectionIdx, double>>, Compare> to_visit;
    to_visit.push({start, 0.0});
    // Set up node map
    smallerNode temp1 = {start, (StreetSegmentIdx)NULL, -turn_penalty, false};
    nodes.emplace(start, temp1);
    smallerNode* current = &(nodes.find(start)->second);

    std::set<IntersectionIdx> targets_set;
    for (auto &it : targets) { targets_set.emplace(it); }

    // Main loop
    while (!targets_set.empty() && to_visit.size() > 0) {
        IntersectionIdx current_id = to_visit.top().first;
        current = &(nodes.find(current_id)->second);
        to_visit.pop();
        if (current->visited) { continue; }

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
            if (getStreetSegmentInfo(current->prev_segment).streetID != getStreetSegmentInfo(branch.first).streetID) {
                turn_cost = turn_penalty;
            }
            double time = findDriveTime(turn_cost, current->time, findStreetSegmentTravelTime(branch.first));
            auto intersection = nodes.find(branch.second);
            // Intersection never viewed -- create entry
            if (intersection == nodes.end()) {
                smallerNode temp2 = {branch.second, branch.first, time, false};
                nodes.emplace(branch.second, temp2);
                to_visit.push({branch.second, time});
            // Intersection has been viewed and current route is better than existing
            } else if (intersection->second.time > time) {
                intersection->second.time = time;
                intersection->second.prev_segment = branch.first;
                to_visit.push({branch.second, intersection->second.time});
            }
        }
        current->visited = true;
        targets_set.erase(current_id);

    }
    std::vector<double> times;
    times.resize(targets.size());
    for (int i = 0; i < targets.size(); i++) {
        if (targets[i] == start) { times[i] = 0; }
        else { times[i] = nodes.find(targets[i])->second.time; }
    }
    return times;
}

// Returns a matrix (2d vector) of travel times between each intersection
/* The matrix is ordered as follows: (A = pickup, B = dropoff, D = depot)
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
 */
std::vector<std::vector<double>> buildTravelTimeMatrix(const double turn_penalty, 
                                                       const std::vector<DeliveryInf>& deliveries, 
                                                       const std::vector<IntersectionIdx>& depots) {
    std::vector<IntersectionIdx> ordered_intersections;
    for (auto &it : deliveries) { ordered_intersections.push_back(it.pickUp); }
    for (auto &it : deliveries) { ordered_intersections.push_back(it.dropOff); }
    for (auto &it : depots)     { ordered_intersections.push_back(it); }

    std::vector<std::vector<double>> matrix;
    for (auto &start : ordered_intersections) {
        matrix.push_back(findTravelTimes(turn_penalty, start, ordered_intersections));
    }

    return matrix;
}

// A function run by a thread for building the travel time matrix
void buildTravelTimeMatrixThread(const double turn_penalty, 
                                 const std::vector<IntersectionIdx>& targets,
                                 std::vector<std::atomic<bool>*>& available_work, 
                                 std::vector<std::vector<double>>& matrix) {
    for (int i = 0; i < available_work.size(); i++) {
        if ((available_work[i])->load(std::memory_order_seq_cst)) {
            (available_work[i])->store(false, std::memory_order_release);
            matrix[i] = findTravelTimes(turn_penalty, targets[i], targets);
            i = -1;
        }
    }
}

// Returns a matrix (2d vector) of travel times between each intersection
std::vector<std::vector<double>> buildTravelTimeMatrixMultiThread(const double turn_penalty, 
                                                                  const std::vector<DeliveryInf>& deliveries, 
                                                                  const std::vector<IntersectionIdx>& depots) {
    std::vector<IntersectionIdx> ordered_intersections;
    for (auto &it : deliveries) { ordered_intersections.push_back(it.pickUp); }
    for (auto &it : deliveries) { ordered_intersections.push_back(it.dropOff); }
    for (auto &it : depots)     { ordered_intersections.push_back(it); }

    std::vector<std::vector<double>> matrix;
    matrix.resize(ordered_intersections.size());

    // For threads to check for what to work on next
    std::vector<std::atomic<bool>*> available_work;
    for (int i = 0; i < ordered_intersections.size(); i++) {
        std::atomic<bool>* temp = new std::atomic<bool>(true);
        available_work.push_back(temp);
    }

    // Create multiple threads
    std::vector<std::thread> threads;
    for (int i = 0; i < ordered_intersections.size() && i < std::thread::hardware_concurrency(); i++) {
        threads.push_back(std::thread(buildTravelTimeMatrixThread, turn_penalty, std::ref(ordered_intersections), std::ref(available_work), std::ref(matrix)));
    }

    // Clean up threads
    for (auto &it : threads) { it.join(); }
    // Deallocate memory
    for (auto &it : available_work) { delete it; }

    return matrix;
}

// Returns a map that can be used to convert an intersection id to an index to use with the matrix
std::map<IntersectionIdx, int> buildMatrixSearchMap(const std::vector<DeliveryInf>& deliveries,
                                                    const std::vector<IntersectionIdx>& depots) {
    std::map<IntersectionIdx, int> search_map;
    int i = 0;
    for (auto &it : deliveries) { search_map.emplace(it.pickUp, i++); }
    for (auto &it : deliveries) { search_map.emplace(it.dropOff, i++); }
    for (auto &it : depots)     { search_map.emplace(it, i++); }
    return search_map;
}

// Uses the travel time matrix and its map to get the travel time between two intersections
double getTimeBetweenIntersections(const std::pair<IntersectionIdx, IntersectionIdx>& intersections,
                                   const std::vector<std::vector<double>>& matrix, 
                                   const std::map<IntersectionIdx, int>& search_map) {
    int idx1 = search_map.find(intersections.first)->second;
    int idx2 = search_map.find(intersections.second)->second;
    return matrix[idx1][idx2];
}

// Returns the total matrix travel time of a stop order
// This only uses the precomputed matrix, so it is fast
// If the stop order is too short to represent a route,
// return infinity so it will never be chosen as the best candidate
double computeStopOrderTravelTime(
    const std::vector<IntersectionIdx>& stop_order,
    const std::vector<std::vector<double>>& matrix,
    const std::map<IntersectionIdx, int>& search_map) {

    if (stop_order.size() < 2) {
        return std::numeric_limits<double>::infinity();
    }

    double total_time = 0.0;

    // Sum the matrix travel time for each consecutive pair of stops
    for (size_t i = 0; i + 1 < stop_order.size(); i++) {
        total_time += getTimeBetweenIntersections(
            {stop_order[i], stop_order[i + 1]},
            matrix,
            search_map
        );
    }

    return total_time;
}


void optimizationThread(std::vector<IntersectionIdx>& route,
                        const std::vector<DeliveryInf>& deliveries, 
                        const std::vector<std::vector<double>>& time_matrix, 
                        const std::map<IntersectionIdx, int>& search_map,
                        const double allotted_time) {
    std::vector<IntersectionIdx> new_route = route;
    // different optimization techniques utilize 100% of the time remaining
    //swap optimization to utilize 20% of remaining time
    new_route = optimizeViaSwap(new_route, deliveries, time_matrix, search_map, allotted_time * 0.2);
    //or_opt to utilize 40% of time
    new_route = optimizeViaRelocate(new_route, deliveries, time_matrix, search_map, allotted_time * 0.4);
    //2-opt to utilize 40% of time 
    new_route = optimizeVia2_Opt(new_route, deliveries, time_matrix, search_map, allotted_time * 0.4);
    
    route = new_route;
}



//// optimizes route heuristicly by 2_opt method
std::vector<IntersectionIdx> optimizeVia2_Opt(
    std::vector<IntersectionIdx>& route,
    const std::vector<DeliveryInf>& deliveries,
    const std::vector<std::vector<double>>& matrix,
    const std::map<IntersectionIdx, int>& search_map,
    double allotted_time
) {
    //record the start time of this function to ensure it only runs for allotted time
    auto start_time = std::chrono::high_resolution_clock::now();

    int route_length = route.size();

    // store the intersetions and their order in an unordered set for later use
    std::unordered_map<IntersectionIdx, int> position;
    for (int stop_point = 0; stop_point < route_length; stop_point++) {
        position[route[stop_point]] = stop_point;
    }

    auto current_time = std::chrono::high_resolution_clock::now();
    while (std::chrono::duration<double, std::milli>(current_time - start_time).count() < allotted_time) {
        current_time = std::chrono::high_resolution_clock::now();

        //select a random stopping point index
        // ignore intersections that are depots

        int first_intersection = 1 + rand() % (route_length - 3);

        //sample 5 different intersections with highest travel-times (A->B) to apply reversing on
        for (int t = 0; t < 5; t++) {
            int candidate = 1 + rand() % (route_length - 3);

            double curr_edge =
                getTimeBetweenIntersections({route[first_intersection], route[first_intersection+1]}, matrix, search_map);

            double cand_edge =
                getTimeBetweenIntersections({route[candidate], route[candidate+1]}, matrix, search_map);

            if (cand_edge > curr_edge) {
                first_intersection = candidate;
            }
        }
        int max_range = route_length - first_intersection - 3;  //3 ensures the intersection isnt close to theend depot
        if (max_range <= 0) continue;

        //determine the second intersection randomly while in the range
        int second_intersection = first_intersection + 2 + rand() % max_range;
        /////

        // determine the travel-time of the affected section
        // if its less then the route is more efficient
        double before =
            getTimeBetweenIntersections({route[first_intersection-1], route[first_intersection]}, matrix, search_map) +
            getTimeBetweenIntersections({route[second_intersection], route[second_intersection+1]}, matrix, search_map);

        double after =
            getTimeBetweenIntersections({route[first_intersection-1], route[second_intersection]}, matrix, search_map) +
            getTimeBetweenIntersections({route[first_intersection], route[second_intersection+1]}, matrix, search_map);

        if (after >= before) continue; // not improving

        // check if the path is legal
        // check if all deliveries of the affected route segment are done after their pickups
        bool valid = true;

        for (const auto& d : deliveries) {
            int pickup = position[d.pickUp];
            int drop = position[d.dropOff];

            //conditions for the path being legal
            bool p_in = (pickup >= first_intersection && pickup <= second_intersection);
            bool d_in = (drop >= first_intersection && drop <= second_intersection);

            int new_p = pickup;
            int new_d = drop;

            if (p_in) new_p = first_intersection + second_intersection - pickup;
            if (d_in) new_d = first_intersection + second_intersection - drop;

            // if pickup is done after dropoff, path is not valid
            if (new_p > new_d) {
                valid = false;
                break;
            }
        }

        if (!valid) continue;

        // reverse the direction between first and second intersections picked (A->B ==> A<-B)
        std::reverse(route.begin() + first_intersection, route.begin() + second_intersection + 1);

        // update position map
        for (int k = first_intersection; k <= second_intersection; k++) {
            position[route[k]] = k;
        }
    }

    return route;
}



/// optimizes route heuristicly via swap method
std::vector<IntersectionIdx> optimizeViaSwap(
    std::vector<IntersectionIdx>& route,
    const std::vector<DeliveryInf>& deliveries,
    const std::vector<std::vector<double>>& matrix,
    const std::map<IntersectionIdx, int>& search_map,
    double allotted_time
) {
    //cache the time the function starts
    //cache the route length
    auto start_time = std::chrono::high_resolution_clock::now();
    int route_length = route.size();

    //record the intersections and their order in an unordered map
    std::unordered_map<IntersectionIdx, int> position;
    for (int stop_point = 0; stop_point < route_length; stop_point++) {
        position[route[stop_point]] = stop_point;
    }

    //ensure the optimization runs for the allotted time amount
    auto current_time = std::chrono::high_resolution_clock::now();
    while (std::chrono::duration<double, std::milli>(current_time - start_time).count() < allotted_time) {
        current_time = std::chrono::high_resolution_clock::now();
        
        // pick 2 random intersections while avoiding depots
        int first_intersection = 1 + std::rand() % (route_length - 2);
        int second_intersection = 1 + std::rand() % (route_length - 2);

        // if the intersections are the same, avoid them
        if (first_intersection == second_intersection) continue;
        if (first_intersection > second_intersection) std::swap(first_intersection, second_intersection);

        
        double before = 0.0;
        double after = 0.0;

        // record the travel time from-to neighboring intersection of the random intersections
        //record the travel time of the affected route segment before swapping
        if (first_intersection > 0)
            before += getTimeBetweenIntersections({route[first_intersection-1], route[first_intersection]}, matrix, search_map);
        if (first_intersection < route_length-1)
            before += getTimeBetweenIntersections({route[first_intersection], route[first_intersection+1]}, matrix, search_map);
        if (second_intersection > 0)
            before += getTimeBetweenIntersections({route[second_intersection-1], route[second_intersection]}, matrix, search_map);
        if (second_intersection < route_length-1)
            before += getTimeBetweenIntersections({route[second_intersection], route[second_intersection+1]}, matrix, search_map);

        // simulate swap locally
        IntersectionIdx sim_first = route[first_intersection];
        IntersectionIdx sim_second = route[second_intersection];

        // record the travel time from-to neighboring intersection of the random intersections
        //record the travel time of the affected route segment after swapping
        if (first_intersection > 0)
            after += getTimeBetweenIntersections({route[first_intersection-1], sim_second}, matrix, search_map);
        if (first_intersection < route_length-1)
            after += getTimeBetweenIntersections({sim_second, (first_intersection+1==second_intersection ? sim_first : route[first_intersection+1])}, matrix, search_map);
        if (second_intersection > 0)
            after += getTimeBetweenIntersections({(second_intersection-1==first_intersection ? sim_first : route[second_intersection-1]), sim_first}, matrix, search_map);
        if (second_intersection < route_length-1)
            after += getTimeBetweenIntersections({sim_first, route[second_intersection+1]}, matrix, search_map);

        // if the simulated swapping gave a worse time result, skip
        if (after >= before) continue;

        // checks if the route is legal after swap
        bool valid = true;

        for (const auto& d : deliveries) {
            int pickup = position[d.pickUp];
            int drop = position[d.dropOff];

            int new_p = pickup;
            int new_d = drop;

            if (pickup == first_intersection) new_p = second_intersection;
            else if (pickup == second_intersection) new_p = first_intersection;

            if (drop == first_intersection) new_d = second_intersection;
            else if (drop == second_intersection) new_d = first_intersection;

            if (new_p > new_d) {
                valid = false;
                break;
            }
        }

        if (!valid) continue;

        // apply the swap 
        std::swap(route[first_intersection], route[second_intersection]);

        // update position map
        position[route[first_intersection]] = first_intersection;
        position[route[second_intersection]] = second_intersection;
    }

    return route;
}

//
std::vector<IntersectionIdx> buildRandomizedGreedyStopOrderFromDepot(
    IntersectionIdx start_depot,
    const std::vector<DeliveryInf>& deliveries,
    const std::vector<IntersectionIdx>& depots,
    const std::vector<std::vector<double>>& time_matrix,
    const std::map<IntersectionIdx, int>& intersection_to_matrix_index,
    int K   
) {

    std::vector<IntersectionIdx> stop_order;

    //cache a vector of intersections and their status (pickup, dropoff)
    std::vector<bool> picked_up(deliveries.size(), false);
    std::vector<bool> dropped_off(deliveries.size(), false);

    //cache a vector of legal stops
    std::vector<IntersectionIdx> legal_stops =
        getLegalNextStops(deliveries, picked_up, dropped_off);

    //return of there are no legal stops connecting to this intersection
    if (legal_stops.empty()) return {};

    stop_order.push_back(start_depot);
    IntersectionIdx current_intersection = start_depot;

    //run the loop while until a valid legal route is created
    while (!allDone(dropped_off)) {

        legal_stops = getLegalNextStops(deliveries, picked_up, dropped_off);
        if (legal_stops.empty()) return {};

        int current_index = intersection_to_matrix_index.at(current_intersection);

        //find top K candidates for this intersection 
        std::vector<std::pair<double, IntersectionIdx>> best_candidates;

        //push the best candidates into the candidate vector
        for (size_t stop_point = 0; stop_point < legal_stops.size(); stop_point++) {
            IntersectionIdx candidate = legal_stops[stop_point];
            int idx = intersection_to_matrix_index.at(candidate);

            double t = time_matrix[current_index][idx];

            best_candidates.push_back(std::make_pair(t, candidate));
        }

        // sort candidates by travel time from this intersection
        std::sort(best_candidates.begin(), best_candidates.end());

        // limit to top K candidates
        int limit = std::min(K, (int)best_candidates.size());

        // randomly pick from best candidates
        int chosen = rand() % limit;
        IntersectionIdx next_stop = best_candidates[chosen].second;

        //push back the valid candidate 
        stop_order.push_back(next_stop);
        current_intersection = next_stop;

        processIntersection(current_intersection, deliveries, picked_up, dropped_off);
    }

    IntersectionIdx best_end_depot = depots[0];
    double best_end_time = std::numeric_limits<double>::infinity();

    int current_index = intersection_to_matrix_index.at(current_intersection);
    
    // look for end depot out of all depots with least travel time from other depots
    for (size_t d = 0; d < depots.size(); d++) {
        IntersectionIdx depot = depots[d];
        int depot_index = intersection_to_matrix_index.at(depot);
        double current_time = time_matrix[current_index][depot_index];

        if (current_time < best_end_time) {
            best_end_time = current_time;
            best_end_depot = depot;
        }
    }
    stop_order.push_back(best_end_depot);
    return stop_order;
}


//
std::vector<IntersectionIdx> optimizeViaRelocate(
    std::vector<IntersectionIdx>& route,
    const std::vector<DeliveryInf>& deliveries,
    const std::vector<std::vector<double>>& matrix,
    const std::map<IntersectionIdx, int>& search_map,
    double allotted_time
) {
    auto start_time = std::chrono::high_resolution_clock::now();
    int route_length = route.size();

    if (route_length < 4) return route;

    std::unordered_map<IntersectionIdx, int> position;
    for (int stop_point = 0; stop_point < route_length; stop_point++) {
        position[route[stop_point]] = stop_point;
    }

    auto current_time = std::chrono::high_resolution_clock::now();

    while (std::chrono::duration<double, std::milli>(current_time - start_time).count() < allotted_time) {
        current_time = std::chrono::high_resolution_clock::now();

        // pick 2 random intersections, avoiding the depots
        int first_intersection = 1 + rand() % (route_length - 2);
        int second_intersection = 1 + rand() % (route_length - 2);

        // if the intersections are the same, skip
        if (first_intersection == second_intersection) continue;

        IntersectionIdx stop_point = route[first_intersection];

        // compute the travel_time with neighbors before moving the intersection to another index
        double before =
            getTimeBetweenIntersections({route[first_intersection-1], route[first_intersection]}, matrix, search_map) +
            getTimeBetweenIntersections({route[first_intersection], route[first_intersection+1]}, matrix, search_map) -
            getTimeBetweenIntersections({route[first_intersection-1], route[first_intersection+1]}, matrix, search_map);

        // compute the travel_time with neighbors after moving the intersection to another index
        double after =
            getTimeBetweenIntersections({route[second_intersection-1], stop_point}, matrix, search_map) +
            getTimeBetweenIntersections({stop_point, route[second_intersection]}, matrix, search_map) -
            getTimeBetweenIntersections({route[second_intersection-1], route[second_intersection]}, matrix, search_map);

        //if the travel-time of the affected route segment is more than it was before, skip
        if (after >= before) continue;

        // check if the route is legal

        // Simulate new positions after relocation
        int sim_first = second_intersection;
        bool valid = true;


        for (const auto& d : deliveries) {
            int pickup = position[d.pickUp];
            int drop = position[d.dropOff];

            // adjust the pickup and drop index based on the shift caused by moving the node
            if (pickup > first_intersection) pickup--;
            if (drop > first_intersection) drop--;
            if (pickup >= sim_first) pickup++;
            if (drop >= sim_first) drop++;

            // if moved stop_point is pickup or dropoff itseld
            if (route[first_intersection] == d.pickUp) pickup = sim_first;
            if (route[first_intersection] == d.dropOff) drop = sim_first;

            //if pickup is done after the drop, its not valid route
            if (pickup > drop) {
                valid = false;
                break;
            }
        }

        if (!valid) continue;

        //move the intersection to the index
        route.erase(route.begin() + first_intersection);
    
        if (first_intersection < second_intersection) {
        route.insert(route.begin() + second_intersection - 1, stop_point);
        } else {
            route.insert(route.begin() + second_intersection, stop_point);
        }

        // update the position map
        for (int k = 0; k < route_length; k++) {
            position[route[k]] = k;
        }
    }
    return route;
}
////////////