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
#include <chrono>
using namespace std;

/************ DIJKSTRA INITIALIZER ***********/
unsigned NO_PREV = 69696969;
unsigned NO_EDGE = 99999999;
double bestPathLen = 999999;

/*********************************************/

void load_graph() {
    for (unsigned i = 0; i < getNumberOfIntersections(); i++) {
        Vertex temp_vertex = Vertex(i);
        DataStructure::vector_of_verticies.push_back(temp_vertex);

        vector <unsigned> street_segments = find_intersection_street_segments(i);

        for (auto j = street_segments.begin(); j != street_segments.end(); j++) {
            StreetSegmentInfo temp_info = getStreetSegmentInfo(*j);
            unsigned temp_from = temp_info.from;
            unsigned temp_to = temp_info.to;
            bool temp_oneway = temp_info.oneWay;
            double temp_street_seg_travel_time = find_street_segment_travel_time(*j);

            Edge temp_edge = Edge(temp_from, temp_to, temp_oneway, *j, temp_street_seg_travel_time);

            bool check_if = false;

            vector <Edge> check = DataStructure::vector_of_verticies[i].adjacent_edges;

            for (auto k = check.begin(); k != check.end(); k++) {
                if ((*k).street_seg_id == (*j)) {
                    check_if = true;
                }
            }

            if (check_if == false) {
                DataStructure::vector_of_verticies[i].adjacent_edges.push_back(temp_edge);
            }
        }

    }
}


// Returns a path (route) between the start intersection and the end intersection, if one exists.

vector<unsigned> find_path_between_intersections(unsigned intersect_id_start, unsigned intersect_id_end) {
    
    auto startTime = chrono::high_resolution_clock::now();
    
    list <unsigned> path_rev;
    vector< unsigned> path;
    bool path_found = false;
    path_rev.clear();
    path.clear();

    set_all_node_visited_false(intersect_id_start); // resetting all the nodes and setting the starting node's pathLen to 0

    path_found = shortest_path(intersect_id_start, intersect_id_end); // if a path is found, returns true

    if (path_found) {
        path_rev = bfsTraceBack(intersect_id_end); // if path found, traces back on the routes to return the path
    }

    for (auto i = path_rev.begin(); i != path_rev.end(); i++) {
        path.push_back(*i);
        DataStructure::shortest_path.push_back(*i);
    }

            auto currentTime = chrono::high_resolution_clock::now();
        auto wallClock = chrono::duration_cast<chrono::duration<double>> (currentTime - startTime);
    
        cout << "load; " << wallClock.count() << endl; 
    
    return path;
}

// Returns the time required to travel along the path specified. 

double compute_path_travel_time(const vector<unsigned>& path) {

    double travel_time = 0;
    unsigned prev_street_id;
    unsigned street_id;
    //cout << "aa" << endl;
    for (auto i = path.begin(); i != path.end(); i++) {
        street_id = DataStructure::street_id_of_all_street_seg_id[(*i)]; // getting the street segment info 
        if (i == path.begin()) { // first time 
            prev_street_id = street_id;
            travel_time += find_street_segment_travel_time(*i); // calling m1 function to get travel time 
        } else {
            if (prev_street_id == street_id) {
                travel_time += find_street_segment_travel_time(*i); // if same street 
            } else {
                travel_time += find_street_segment_travel_time(*i); // if taking a turn
                travel_time += 0.25;
                prev_street_id = street_id;
            }
        }
    }

    return travel_time;
}

// Returns a path to the closest POI from the starting intersection id 

vector<unsigned> find_path_to_point_of_interest(unsigned intersect_id_start, string point_of_interest_name) {
    list <unsigned> path_rev;
    vector <unsigned> path;
    unsigned closest_intersection;
    unsigned path_found = 0;

    set_all_node_visited_false(intersect_id_start);
    DataStructure::POI_look_up_table.clear(); // clear

    auto range = DataStructure::POI_names_to_ids.equal_range(point_of_interest_name); // putting all the id's of the POI into a map
    for (auto i = range.first; i != range.second; i++) {

        LatLon temp = DataStructure::lat_lon_of_all_poi_ids[i->second];
        closest_intersection = find_closest_intersection(temp); // find the closest intersections to all the poi id's
        DataStructure::POI_look_up_table.insert({(closest_intersection), (i->second)}); // put in a binary tree with intersection id as key and the poi id as value
    }

    path_found = shortest_path_POI(intersect_id_start); // finds the path and if found, returns the closest poi id 

    if (path_found != 0) { // if path_found isn't 0, a path is found 
        path_rev = bfsTraceBack(path_found); // track back on the routes and return a path
    }

    for (auto i = path_rev.begin(); i != path_rev.end(); i++) {
        path.push_back(*i);
        DataStructure::shortest_path.push_back(*i);
    }

    return path;
}


// Finding if a path exists from an intersection to another intersection 

bool shortest_path(unsigned intersect_id_start, unsigned end_id) {
    priority_queue <waveElem, vector<waveElem>, greater<waveElem> > wavefront;
    wavefront.push(waveElem(DataStructure::vector_of_verticies[intersect_id_start], NO_EDGE, 0, NO_PREV, DataStructure::vector_of_verticies[intersect_id_start].prev_edge));
    unsigned prev_street;
    //clock_t t1, t2, t3, t4, t5, t6;
    // double seconds = 0, seconds1 = 0, seconds2 = 0, seconds3 = 0, seconds4 = 0;

    //t3 = clock();
    while (!wavefront.empty() && wavefront.top().pathLen < bestPathLen) {
        //t5 = clock();
        waveElem curr = wavefront.top(); // get the minimum in the priority queue
        wavefront.pop(); // erase the element 

        unsigned curr_id = curr.vertex.intersection_id;


        
        if (curr.pathLen < DataStructure::vector_of_verticies[curr_id].pathLen) { // if the pathLen of the min of the priority queue is smaller than the node itself

            DataStructure::vector_of_verticies[curr_id].pathLen = curr.pathLen; // put in the values 
            DataStructure::vector_of_verticies[curr_id].reaching_edge_id = curr.edge_id;
            DataStructure::vector_of_verticies[curr_id].prev_edge = curr.edge;

            if (curr.edge_id != 99999999)
            DataStructure::test.push_back(curr.edge_id);
            
            
            if (curr_id == end_id) { // if reached the destination intersection id 
                bestPathLen = DataStructure::vector_of_verticies[curr_id].pathLen;
                //cout << "For loop:"<<seconds << endl;
                //cout << "Init: " << seconds4 << endl;
                //t4 = clock();
                //seconds2 = ( t4 - t3 ) / (double) CLOCKS_PER_SEC;
                //cout << "Total: " << seconds2 << endl;
                return true; // best path is found
            }
            //t6 = clock();
            // seconds3 = ( t6 - t5 ) / (double) CLOCKS_PER_SEC;
            // seconds4 = seconds4 + seconds3;
            // t1 = clock();

            for (auto i = DataStructure::vector_of_verticies[curr_id].adjacent_edges.begin(); i != DataStructure::vector_of_verticies[curr_id].adjacent_edges.end(); i++) {
                prev_street = DataStructure::street_id_of_all_street_seg_id[(*i).street_seg_id];
                if ((*i).oneway == false) { // if not one way
                    if ((*i).from == curr_id) { // if the current node is the adjacent edge's from 
                        if ((DataStructure::vector_of_verticies[curr_id].pathLen + (*i).street_seg_travel_time) < DataStructure::vector_of_verticies[(*i).to].pathLen) {
                            unsigned to = (*i).to;
                            Vertex to_vertex = DataStructure::vector_of_verticies[to];
                            if (prev_street == curr.previous_street || curr.previous_street == NO_PREV) { // if same street, don't add 0.25 (15seconds)
                                wavefront.push(waveElem(to_vertex, (*i).street_seg_id, DataStructure::vector_of_verticies[curr_id].pathLen + (*i).street_seg_travel_time, prev_street, (*i)));
                            } else {// if different street, add 0.25 (15seconds)
                                wavefront.push(waveElem(to_vertex, (*i).street_seg_id, DataStructure::vector_of_verticies[curr_id].pathLen + 0.25 + (*i).street_seg_travel_time, prev_street, (*i)));
                            }
                        }
                    } else { // if the current node is the adjacent edge's to 
                        if ((DataStructure::vector_of_verticies[curr_id].pathLen + (*i).street_seg_travel_time) < DataStructure::vector_of_verticies[(*i).from].pathLen) {
                            Vertex from_vertex = DataStructure::vector_of_verticies[(*i).from];
                            if (prev_street == curr.previous_street || curr.previous_street == NO_PREV) { // if same street, don't add 0.25 (15seconds)
                                wavefront.push(waveElem(from_vertex, (*i).street_seg_id, DataStructure::vector_of_verticies[curr_id].pathLen + (*i).street_seg_travel_time, prev_street, (*i)));
                            } else { // if different street, add 0.25 (15seconds)
                                wavefront.push(waveElem(from_vertex, (*i).street_seg_id, DataStructure::vector_of_verticies[curr_id].pathLen + 0.25 + (*i).street_seg_travel_time, prev_street, (*i)));
                            }
                        }
                    }
                } else {

                    if ((*i).oneway == true && (*i).from == curr_id) {
                        if ((DataStructure::vector_of_verticies[curr_id].pathLen + (*i).street_seg_travel_time) < DataStructure::vector_of_verticies[(*i).to].pathLen) {
                            unsigned to = (*i).to;
                            Vertex to_vertex = DataStructure::vector_of_verticies[to];
                            if (prev_street == curr.previous_street || curr.previous_street == NO_PREV) { // if same street, don't add 0.25 (15seconds)
                                wavefront.push(waveElem(to_vertex, (*i).street_seg_id, DataStructure::vector_of_verticies[curr_id].pathLen + (*i).street_seg_travel_time, prev_street, (*i)));
                            } else { // if different street, add 0.25 (15seconds)
                                wavefront.push(waveElem(to_vertex, (*i).street_seg_id, DataStructure::vector_of_verticies[curr_id].pathLen + 0.25 + (*i).street_seg_travel_time, prev_street, (*i)));
                            }
                        }
                    }
                }
            }

            // t2 = clock();

            //seconds1 = ( t2 - t1 ) / (double) CLOCKS_PER_SEC;
            // seconds = seconds + seconds1;

        }
    }
    return false;
}


// Tracks back on the shortest route found and returns a vector of actual path 

list<unsigned> bfsTraceBack(unsigned end_id) {

    list <unsigned> path;

    unsigned prev_edge = DataStructure::vector_of_verticies[end_id].reaching_edge_id;
    unsigned prev_from;
    prev_from = end_id;
    unsigned set_node = end_id;
    while (prev_edge != NO_EDGE) {
        path.push_front(prev_edge); // put in vector 
        //DataStructure::shortest_path.push_back(prev_edge);
        Edge previous = DataStructure::vector_of_verticies[prev_from].prev_edge;

        if (previous.oneway == true) { // if one-way, from is from 
            prev_from = previous.from;
        } else {
            if (set_node == previous.from) { // if not one-way, and the last node visited is the from, previous from is the to
                prev_from = previous.to;
            } else
                prev_from = previous.from; // if not one-way, and the last node visited is the to, previous from is the from
        }
        prev_edge = DataStructure::vector_of_verticies[prev_from].reaching_edge_id; // getting the reaching edge id of the previous from 
        set_node = prev_from; // current node == set_node
    }

    //reverse(path.begin(), path.end()); // since started from the end, the vector is backwards, needs to reverse
    //reverse (DataStructure::shortest_path.begin(), DataStructure::shortest_path.end());
    return path;
}

void set_all_node_visited_false(unsigned intersect_id_start) {

    for (unsigned i = 0; i < getNumberOfIntersections(); i++) {

        if (i == intersect_id_start)
            DataStructure::vector_of_verticies[i].pathLen = 0; // if the node is the start, set the pathLen to 0
        unsigned length = std::numeric_limits<unsigned>::max();
        DataStructure::vector_of_verticies[i].pathLen = length; // rest of the nodes to INF
        DataStructure::vector_of_verticies[i].reaching_edge_id = -1; // reaching edge to -1 

    }
    bestPathLen = 999999;
}

// Finding if a path exists from an intersection to a POI 
unsigned shortest_path_POI(unsigned intersect_id_start) {

    unsigned found = 0;

    map<unsigned, unsigned>::iterator iter;

    priority_queue <waveElem, vector<waveElem>, greater<waveElem> > wavefront;
    wavefront.push(waveElem(DataStructure::vector_of_verticies[intersect_id_start], NO_EDGE, 0, NO_PREV, DataStructure::vector_of_verticies[intersect_id_start].prev_edge));

    unsigned prev_street;

    while (!wavefront.empty() && wavefront.top().pathLen < bestPathLen) {
        waveElem curr = wavefront.top(); // get the min
        wavefront.pop(); // erase 

        unsigned curr_id = curr.vertex.intersection_id;

        if (curr.pathLen < DataStructure::vector_of_verticies[curr_id].pathLen) { // if the pathLen of the min of the priority queue is smaller than the node itself

            DataStructure::vector_of_verticies[curr_id].pathLen = curr.pathLen;
            DataStructure::vector_of_verticies[curr_id].reaching_edge_id = curr.edge_id;
            DataStructure::vector_of_verticies[curr_id].prev_edge = curr.edge;

            iter = DataStructure::POI_look_up_table.find(curr_id); // see if the current node is one of the intersection id's that are closest to the POI's

            if (iter != DataStructure::POI_look_up_table.end()) { // if the current node is the closest intersection id of the closest poi to the start intersection id
                bestPathLen = DataStructure::vector_of_verticies[curr_id].pathLen;
                found = curr_id; // set it to true and continue 
            }
            for (auto i = DataStructure::vector_of_verticies[curr_id].adjacent_edges.begin(); i != DataStructure::vector_of_verticies[curr_id].adjacent_edges.end(); i++) {
                prev_street = DataStructure::street_id_of_all_street_seg_id[(*i).street_seg_id];

                if ((*i).oneway == false) { // if not one way
                    if ((*i).from == curr_id) {
                        if ((DataStructure::vector_of_verticies[curr_id].pathLen + (*i).street_seg_travel_time) < DataStructure::vector_of_verticies[(*i).to].pathLen) {
                            unsigned to = (*i).to;
                            Vertex to_vertex = DataStructure::vector_of_verticies[to];
                            if (prev_street == curr.previous_street || curr.previous_street == NO_PREV) // if same street, don't add 15 seconds 
                                wavefront.push(waveElem(to_vertex, (*i).street_seg_id, DataStructure::vector_of_verticies[curr_id].pathLen + (*i).street_seg_travel_time, prev_street, (*i)));
                            else // if different street, add 15 seconds 
                                wavefront.push(waveElem(to_vertex, (*i).street_seg_id, DataStructure::vector_of_verticies[curr_id].pathLen + 0.25 + (*i).street_seg_travel_time, prev_street, (*i)));
                        }
                    } else {
                        if ((DataStructure::vector_of_verticies[curr_id].pathLen + (*i).street_seg_travel_time) < DataStructure::vector_of_verticies[(*i).from].pathLen) {
                            Vertex from_vertex = DataStructure::vector_of_verticies[(*i).from];
                            if (prev_street == curr.previous_street || curr.previous_street == NO_PREV) // if same street, don't add 15 seconds 
                                wavefront.push(waveElem(from_vertex, (*i).street_seg_id, DataStructure::vector_of_verticies[curr_id].pathLen + (*i).street_seg_travel_time, prev_street, (*i)));
                            else // if different street, add 15 seconds 
                                wavefront.push(waveElem(from_vertex, (*i).street_seg_id, DataStructure::vector_of_verticies[curr_id].pathLen + 0.25 + (*i).street_seg_travel_time, prev_street, (*i)));
                        }
                    }
                } else {
                    if ((*i).oneway == true && (*i).from == curr_id) {
                        // if one way and is in the right way
                        if ((DataStructure::vector_of_verticies[curr_id].pathLen + (*i).street_seg_travel_time) < DataStructure::vector_of_verticies[(*i).to].pathLen) {

                            unsigned to = (*i).to;
                            Vertex to_vertex = DataStructure::vector_of_verticies[to];
                            if (prev_street == curr.previous_street || curr.previous_street == NO_PREV) // if same street, don't add 15 seconds 
                                wavefront.push(waveElem(to_vertex, (*i).street_seg_id, DataStructure::vector_of_verticies[curr_id].pathLen + (*i).street_seg_travel_time, prev_street, (*i)));
                            else // if different street, add 15 seconds 
                                wavefront.push(waveElem(to_vertex, (*i).street_seg_id, DataStructure::vector_of_verticies[curr_id].pathLen + 0.25 + (*i).street_seg_travel_time, prev_street, (*i)));
                        }
                    }
                }
            }
        }
    }

    return found;
}



/******************************** MILESTONE 4 ******************************************************/
//vector<unsigned> multi_destination(unsigned intersect_id_start, const std::vector<DeliveryInfo>& deliveries) {
//    list <unsigned> path_rev;
//    vector <unsigned> path;
//    unsigned closest_intersection;
//    unsigned path_found = 0;
//
//    set_all_node_visited_false(intersect_id_start);
//    DataStructure::pick_up_look_up_table.clear(); // clear
//    
//    for (auto i = deliveries.begin(); i != deliveries.end(); i++) {
//        DataStructure::pick_up_look_up_table.insert({(*i).pickUp, (*i).dropOff});
//    }
//
//    //path_found = multi_path(intersect_id_start);
//    
//    if (path_found != 0){
//        path_rev = bfsTraceBack(path_found);
//    }
//    
//    
//    
////    path_found = shortest_path_POI(intersect_id_start); // finds the path and if found, returns the closest poi id 
////
////    if (path_found != 0) { // if path_found isn't 0, a path is found 
////        path_rev = bfsTraceBack(path_found); // track back on the routes and return a path
////    }
////    
////    for (auto i = path_rev.begin(); i != path_rev.end(); i++){
////        path.push_back(*i);
////        DataStructure::shortest_path.push_back(*i);
////    }
////    
////    return path;
//}
//
//unsigned multi_path(unsigned intersect_id_start) {
//
//    unsigned found = 0;
//
//    map<unsigned, unsigned>::iterator iter;
//
//    priority_queue <waveElem, vector<waveElem>, greater<waveElem>  > wavefront;
//    wavefront.push(waveElem(DataStructure::vector_of_verticies[intersect_id_start], NO_EDGE, 0, NO_PREV, DataStructure::vector_of_verticies[intersect_id_start].prev_edge));
//
//    unsigned prev_street;
//
//    while (!wavefront.empty() && wavefront.top().pathLen < bestPathLen) {
//        waveElem curr = wavefront.top(); // get the min
//        wavefront.pop(); // erase 
//
//        unsigned curr_id = curr.vertex.intersection_id;
//
//        if (curr.pathLen < DataStructure::vector_of_verticies[curr_id].pathLen) { // if the pathLen of the min of the priority queue is smaller than the node itself
//
//            DataStructure::vector_of_verticies[curr_id].pathLen = curr.pathLen;
//            DataStructure::vector_of_verticies[curr_id].reaching_edge_id = curr.edge_id;
//            DataStructure::vector_of_verticies[curr_id].prev_edge = curr.edge;
//
//            iter = DataStructure::POI_look_up_table.find(curr_id); // see if the current node is one of the intersection id's that are closest to the POI's
//
//            if (iter != DataStructure::POI_look_up_table.end()) { // if the current node is the closest intersection id of the closest poi to the start intersection id
//                bestPathLen = DataStructure::vector_of_verticies[curr_id].pathLen;
//                found = curr_id; // set it to true and continue 
//            }
//            for (auto i = DataStructure::vector_of_verticies[curr_id].adjacent_edges.begin(); i != DataStructure::vector_of_verticies[curr_id].adjacent_edges.end(); i++) {
//                prev_street = DataStructure::street_id_of_all_street_seg_id[(*i).street_seg_id];
//
//                if ((*i).oneway == false) { // if not one way
//                    if ((*i).from == curr_id) {
//                        if ((DataStructure::vector_of_verticies[curr_id].pathLen + (*i).street_seg_travel_time) < DataStructure::vector_of_verticies[(*i).to].pathLen) {
//                            unsigned to = (*i).to;
//                            Vertex to_vertex = DataStructure::vector_of_verticies[to];
//                            if (prev_street == curr.previous_street || curr.previous_street == NO_PREV) // if same street, don't add 15 seconds 
//                                wavefront.push(waveElem(to_vertex, (*i).street_seg_id, DataStructure::vector_of_verticies[curr_id].pathLen + (*i).street_seg_travel_time, prev_street, (*i)));
//                            else // if different street, add 15 seconds 
//                                wavefront.push(waveElem(to_vertex, (*i).street_seg_id, DataStructure::vector_of_verticies[curr_id].pathLen + 0.25 + (*i).street_seg_travel_time, prev_street, (*i)));
//                        }
//                    } else {
//                        if ((DataStructure::vector_of_verticies[curr_id].pathLen + (*i).street_seg_travel_time) < DataStructure::vector_of_verticies[(*i).from].pathLen) {
//                            Vertex from_vertex = DataStructure::vector_of_verticies[(*i).from];
//                            if (prev_street == curr.previous_street || curr.previous_street == NO_PREV) // if same street, don't add 15 seconds 
//                                wavefront.push(waveElem(from_vertex, (*i).street_seg_id, DataStructure::vector_of_verticies[curr_id].pathLen + (*i).street_seg_travel_time, prev_street, (*i)));
//                            else // if different street, add 15 seconds 
//                                wavefront.push(waveElem(from_vertex, (*i).street_seg_id, DataStructure::vector_of_verticies[curr_id].pathLen + 0.25 + (*i).street_seg_travel_time, prev_street, (*i)));
//                        }
//                    }
//                } else {
//                    if ((*i).oneway == true && (*i).from == curr_id) {
//                        // if one way and is in the right way
//                        if ((DataStructure::vector_of_verticies[curr_id].pathLen + (*i).street_seg_travel_time) < DataStructure::vector_of_verticies[(*i).to].pathLen) {
//
//                            unsigned to = (*i).to;
//                            Vertex to_vertex = DataStructure::vector_of_verticies[to];
//                            if (prev_street == curr.previous_street || curr.previous_street == NO_PREV) // if same street, don't add 15 seconds 
//                                wavefront.push(waveElem(to_vertex, (*i).street_seg_id, DataStructure::vector_of_verticies[curr_id].pathLen + (*i).street_seg_travel_time, prev_street, (*i)));
//                            else // if different street, add 15 seconds 
//                                wavefront.push(waveElem(to_vertex, (*i).street_seg_id, DataStructure::vector_of_verticies[curr_id].pathLen + 0.25 + (*i).street_seg_travel_time, prev_street, (*i)));
//                        }
//                    }
//                }
//            }
//        }
//    }
//
//    return found;
//}