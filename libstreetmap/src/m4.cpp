
#include "m4.h"
#include "m4Helpers.h"
#include <map>
#include <thread>
#include <chrono>
#include <limits>
#include <ctime>
#include <cstdlib>

std::vector<CourierSubPath> travelingCourier(const float turn_penalty,
                                             const std::vector<DeliveryInf>& deliveries,
                                             const std::vector<IntersectionIdx>& depots) {

    auto t0 = std::chrono::high_resolution_clock::now();

    // Build the travel-time matrix between the important intersections
    std::vector<std::vector<double>> time_matrix =
        buildTravelTimeMatrixMultiThread(turn_penalty, deliveries, depots);

    // Build the lookup map from intersection id -> matrix index
    std::map<IntersectionIdx, int> intersection_to_matrix_index = buildMatrixSearchMap(deliveries, depots);

    auto t1 = std::chrono::high_resolution_clock::now();
    std::chrono::duration<double, std::milli> time0 = (t1 - t0);
    std::cout << "Travel time matrix built in " << time0.count() << " ms." << std::endl;

    // Try one greedy route per depot and keep the best candidate
    // that can actually be converted into CourierSubPaths
    std::srand(std::time({}));

    std::multimap<double, std::vector<IntersectionIdx>> stop_orders;

    for (size_t i = 0; i < depots.size(); i++) {

    IntersectionIdx start_depot = depots[i];

    // run 2 different variations per depot to increase coverage
    // 2 can be adjusted to another number, but 3+ failed time test
    for (int r = 0; r < 2; r++) {

        std::vector<IntersectionIdx> candidate_stop_order;
        
        // produce routes via different techniques to more coverage
        //generates a greedy route if r == 0
        if (r == 0) {
            candidate_stop_order = buildGreedyStopOrderFromDepot(
                start_depot,
                deliveries,
                depots,
                time_matrix,
                intersection_to_matrix_index
            );
        // generates a randomized greedy route if r == 0
        } else {
            candidate_stop_order = buildRandomizedGreedyStopOrderFromDepot(
                start_depot,
                deliveries,
                depots,
                time_matrix,
                intersection_to_matrix_index,
                4 
            );
        }

        // Skip failed candidates
        if (candidate_stop_order.empty()) {
            continue;
        }

        // Build courier path (validity check)
        std::vector<CourierSubPath> candidate_route =
            buildCourierSubPaths(turn_penalty, candidate_stop_order);

        if (candidate_route.empty()) {
            continue;
        }

        // Score candidate
        double candidate_time = computeStopOrderTravelTime(
            candidate_stop_order,
            time_matrix,
            intersection_to_matrix_index
        );
        
        //store the stopping point
        stop_orders.insert({candidate_time, candidate_stop_order});

        // to maintain performance, limit the number of candidates to check
        const int MAX_CANDIDATES = 20;

        //keep the candidates with best times based on MAX_CANDIDATES 
        if ((int)stop_orders.size() > MAX_CANDIDATES) {
            auto worst = std::prev(stop_orders.end());
            stop_orders.erase(worst);
        }
    }
}


    // If all multi-start candidates failed, return empty
    if (stop_orders.empty() == true) {
        std::cout << "Error: no valid path was created." << std::endl;
        return {};
    }

    // Optimize only the best valid multi-start candidate
    auto t2 = std::chrono::high_resolution_clock::now();
    std::chrono::duration<double, std::milli> time1 = (t2 - t1);
    std::cout << "Initial stop orders " << time1.count() << " ms." << std::endl;

    // Run optimization on several candidates
    std::vector<std::thread> threads;
    auto stop_orders_iter = stop_orders.begin();
    double remaining_time = WORK_STOP_BUFFER_TIME - ((std::chrono::duration<double, std::milli>)(std::chrono::high_resolution_clock::now() - t0)).count();
    // I don't want to wait forever for test cases
    if (deliveries.size() < 50) { remaining_time = 7.50; }
    int i = 0;
    for (; i < stop_orders.size() && i < std::thread::hardware_concurrency(); i++) {
        threads.push_back(std::thread(optimizationThread, std::ref((stop_orders_iter++)->second), std::ref(deliveries), std::ref(time_matrix), std::ref(intersection_to_matrix_index), remaining_time));
    }
    // Wait for threads
    for (auto &thread : threads) { thread.join(); }
    // Retrieve optimized routes
    std::vector<std::vector<IntersectionIdx>> optimized_stop_orders;
    stop_orders_iter = stop_orders.begin();
    for (int j = 0; j < i; j++) {
        optimized_stop_orders.push_back((stop_orders_iter++)->second);
    }
    // Sort optimized routes
    std::pair<int, double> best_route = {0, computeStopOrderTravelTime(optimized_stop_orders[0], time_matrix, intersection_to_matrix_index)};
    for (int k = 0; k < optimized_stop_orders.size(); k++) {
        double travel_time = computeStopOrderTravelTime(optimized_stop_orders[k], time_matrix, intersection_to_matrix_index);
        if (travel_time < best_route.second) { best_route = {k, travel_time}; }
    }
    std::vector<IntersectionIdx> stop_order = optimized_stop_orders[best_route.first];

    auto t3 = std::chrono::high_resolution_clock::now();
    std::chrono::duration<double, std::milli> time2 = (t3 - t0);
    std::cout << "Finished optimization at " << time2.count() << " ms." << std::endl;
        
    // If route construction failed, return empty
    if (stop_order.empty() == true) {
        std::cout << "Error: route construction failed." << std::endl;
        return {};
    }

    // Convert the stop order into the required CourierSubPath output
    std::vector<CourierSubPath> courier_route = buildCourierSubPaths(turn_penalty, stop_order);

    // If path conversion failed, return empty
    if (courier_route.empty() == true) {
        std::cout << "Error: path conversion failed." << std::endl;
        return {};
    }

    return courier_route;
}