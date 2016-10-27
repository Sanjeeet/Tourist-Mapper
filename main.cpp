#include "StreetsDatabaseAPI.h"
#include "m1.h"
#include "iostream"
#include <string>
#include <vector>
#include <algorithm>
#include "OSMDatabaseAPI.h"
#include "m2.h"
#include "data_structure.h"
#include "m3.h"


#include "milestone_3_graph.h"
#include "m3.h"


using namespace std;

int main() {

    string temp;
    cout << "Select which map to load: ";
    getline(cin, temp);

    if (temp == "Toronto") {
        load_map("/cad2/ece297s/public/maps/toronto.streets.bin");
    } else if (temp == "NewYork") {
        load_map("/cad2/ece297s/public/maps/newyork.streets.bin");
    } else if (temp == "Moscow") {
        load_map("/cad2/ece297s/public/maps/moscow.streets.bin");
    } else if (temp == "Cairo") {
        load_map("/cad2/ece297s/public/maps/cairo_egypt.streets.bin");
    } else if (temp == "Saint") {
        load_map("/cad2/ece297s/public/maps/saint_helena.streets.bin");
    }
    else if (temp == "London") {
        load_map("/cad2/ece297s/public/maps/london_england.streets.bin");
    }

    draw_map();
    

    closeOSMDatabase();

    close_map();

    return 0;
}
