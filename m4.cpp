#include "m1.h"
#include "m2.h"
#include "m3.h"
#include <vector>
#include "graphics.h"
#include <cfloat>
#include "StreetsDatabaseAPI.h"
#include "OSMEntity.hpp"
#include <iomanip>
#include <algorithm> 
#include "OSMDatabaseAPI.h"
#include <sstream>
#include "data_structure.h"
#include "milestone_3_graph.h"
#include <list>
#include <limits>
#include <queue>
#include <functional>
#include <time.h>
#include <unordered_map>
#include "m4.h"
#include <map>
#include <chrono>
#include <random>
#include <iterator>

#define TIME_LIMIT 30

vector<unsigned> traveling_courier(const std::vector<DeliveryInfo>& deliveries, const std::vector<unsigned>& depots) {

    unsigned start_depot = 0;
    vector <unsigned> final_path;
    vector <unsigned> temp_path;
    vector <unsigned> current_path;
    double best_travel_time = 9999999; // initializing the best travel time
    double temp_travel_time;
    unsigned counter = 0; // for purpose of breaking the loop after a certain point 

    for (auto k = depots.begin(); k != depots.end(); k++) { // loop for multi-start (trying multiple depots)
        if (counter == 11)
            break; // time runs out if we run all depots, so break once we assess 12 
        DataStructure::shortest_path.clear(); // clearing needed and previously used vectors
        DataStructure::deliveryInfos.clear();
        current_path.clear();

        unsigned start_depot_int_id;
        start_depot_int_id = *k;

        if (DataStructure::which_map == "/cad2/ece297s/public/maps/toronto.osm.bin") { 
            if (*k == 75020) // fixing a bug within the toronto map (street isn't connected to the intersection 75020)
                start_depot_int_id = depots[1];
        }
        cout << start_depot_int_id << " ";
        current_path.push_back(start_depot_int_id); // insert the intersection id of the starting depot into the current_path vector 

        // find closest first pick up to go to
        unsigned closest_int1, closest_int2;
        double distance;
        double closest_distance = 99999;

        for (auto i = deliveries.begin(); i != deliveries.end(); i++) {
            distance = find_distance_between_two_points(DataStructure::lat_lon_of_all_intersection_ids[(*i).pickUp], DataStructure::lat_lon_of_all_intersection_ids[start_depot_int_id ]);

            NewDeliveryInfo tempInfo = NewDeliveryInfo((*i).pickUp, (*i).dropOff, false, false);
            DataStructure::deliveryInfos[(*i).pickUp][(*i).dropOff].push_back(tempInfo);

            if (distance < closest_distance) {
                closest_distance = distance;
                closest_int1 = (*i).pickUp;
                closest_int2 = (*i).dropOff;
            }
        }

        current_path.push_back(closest_int1); // push in the closest pick up into the vector
        DataStructure::deliveryInfos[closest_int1][closest_int2][0].pickedUp = true; // set the boolean to true (it's been picked up) for the pick_up intersection


        unsigned last_int_visited = closest_int1;
        unsigned pick_up_int_id;
        unsigned drop_off_int_id;
        bool pickedUp;
        bool droppedOff;
        unsigned pickUp_or_drop_off;
        unsigned pickUpKey;
        unsigned dropOffKey;
        unsigned num_delievered = 0;

        while (num_delievered != deliveries.size()) {
            closest_distance = 99999999;

            // loop to find the closest possible (legal) intersction id to go to ==> using the euclidian distance
            for (auto i = deliveries.begin(); i != deliveries.end(); i++) {

                pick_up_int_id = (*i).pickUp;
                drop_off_int_id = (*i).dropOff;
                pickedUp = DataStructure::deliveryInfos[pick_up_int_id][drop_off_int_id][0].pickedUp;
                droppedOff = DataStructure::deliveryInfos[pick_up_int_id][drop_off_int_id][0].droppedOff;

                distance = 999999;

                if (pickedUp == false && droppedOff == false) { //not picked up yet

                    distance = find_distance_between_two_points(DataStructure::lat_lon_of_all_intersection_ids[(*i).pickUp], DataStructure::lat_lon_of_all_intersection_ids[last_int_visited]);
                } else if (pickedUp == true && droppedOff == false) { //picked up not dropped off yet

                    distance = find_distance_between_two_points(DataStructure::lat_lon_of_all_intersection_ids[(*i).dropOff], DataStructure::lat_lon_of_all_intersection_ids[last_int_visited]);
                }

                if ((pickedUp == false && droppedOff == false) || (pickedUp == true && droppedOff == false)) {

                    if (distance < closest_distance) { // if the distance is    

                        closest_distance = distance;
                        if (pickedUp == false && droppedOff == false) {

                            closest_int1 = (*i).pickUp;  // 
                            pickUp_or_drop_off = 1;
                            pickUpKey = (*i).pickUp;
                            dropOffKey = (*i).dropOff;
                        } else if (pickedUp == true && droppedOff == false) {

                            closest_int1 = (*i).dropOff;
                            pickUp_or_drop_off = 2;
                            pickUpKey = (*i).pickUp;
                            dropOffKey = (*i).dropOff;
                        }
                    }
                }
            }

            if (pickUp_or_drop_off == 1) { // if it's a pick up 
                current_path.push_back(closest_int1); // push into the current_path vector
                DataStructure::deliveryInfos[pickUpKey][dropOffKey][0].pickedUp = true; // set the boolean
                last_int_visited = closest_int1; // set last intersection id visited to the current intersection id
            } else if (pickUp_or_drop_off == 2) { // if it's a drop off
                current_path.push_back(closest_int1); // push into the current_path vector
                DataStructure::deliveryInfos[pickUpKey][dropOffKey][0].droppedOff = true; // set the boolean
                num_delievered = num_delievered + 1; // after a pick up and a drop off  ==  a full delivery
                last_int_visited = closest_int1; // set last intersection id visited to the current intersection id
            }
        }


        closest_distance = 99999;
        unsigned closest_depot;
        for (auto i = depots.begin(); i != depots.end(); i++) {
            if (DataStructure::which_map == "/cad2/ece297s/public/maps/toronto.osm.bin") { 
                if (*i != 75020) { // fixing a bug within the toronto map (street isn't connected to the intersection 75020)
                    distance = find_distance_between_two_points(DataStructure::lat_lon_of_all_intersection_ids[(*i)], DataStructure::lat_lon_of_all_intersection_ids[last_int_visited]);

                    if (distance < closest_distance) {
                        closest_distance = distance;
                        closest_depot = (*i);
                    }
                }
            }
            else { // if it's other maps
                distance = find_distance_between_two_points(DataStructure::lat_lon_of_all_intersection_ids[(*i)], DataStructure::lat_lon_of_all_intersection_ids[last_int_visited]);

                if (distance < closest_distance) {
                    closest_distance = distance;
                    closest_depot = (*i);
                }
            }
        }


        current_path.push_back(closest_depot); // push in the last depot to the vector of intersections
        temp_path = find_path_segments(current_path); // call our dijkstra function and make path
        temp_travel_time = compute_path_travel_time(temp_path); // compute our path to compare to find the best
        cout << temp_travel_time << endl;
        if (temp_travel_time < best_travel_time) { // path comparison
            best_travel_time = temp_travel_time;
            final_path = temp_path;
        }

        counter++;
    }

    return (final_path);

}

// function to call our dijkstra function to get the actual path

vector<unsigned> find_path_segments(vector<unsigned> current_path) {

    for (unsigned i = 0; i < current_path.size() - 1; i++) {
        find_path_between_intersections(current_path[i], current_path[i + 1]); // call our m3 function N-1 times 
    }

    return (DataStructure::shortest_path);
}