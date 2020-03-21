#include <optional>
#include <fstream>
#include <iostream>
#include <vector>
#include <string>
#include <io2d.h>
#include "route_model.h"
#include "render.h"
#include "route_planner.h"

using namespace std::experimental;

static std::optional<std::vector<std::byte>> ReadFile(const std::string &path)
{   
    std::ifstream is{path, std::ios::binary | std::ios::ate};
    if( !is )
        return std::nullopt;
    
    auto size = is.tellg();
    std::vector<std::byte> contents(size);    
    
    is.seekg(0);
    is.read((char*)contents.data(), size);

    if( contents.empty() )
        return std::nullopt;
    return std::move(contents);
}

void ClearCin() {
    std::cin.clear(); 
    std::cin.ignore(std::numeric_limits<std::streamsize>::max(), '\n');
}

int GetIntFromRange(int lower, int upper){
    int input; 
    while(!(std::cin >> input)) {
        std::cout << "Wrong Input Type. Please try again.\n";
        ClearCin();
   }

    //Bounds control
    while(input < lower || input > upper) {
        std::cout << "Out of Range. Re-enter option: ";
        std::cin.ignore(std::numeric_limits<std::streamsize>::max(), '\n');

        //Second error catch. If out of range integer was entered, and then a non-integer this second one shall catch it
        while(!(std::cin >> input)) {
            std::cout << "Wrong Input Type. Please try again.\n";
            ClearCin();
        }
    }
    return input;
}

int main(int argc, const char **argv)
{    
    std::string osm_data_file = "";
    if( argc > 1 ) {
        for( int i = 1; i < argc; ++i )
            if( std::string_view{argv[i]} == "-f" && ++i < argc )
                osm_data_file = argv[i];
    }
    else {
        std::cout << "To specify a map file use the following format: " << std::endl;
        std::cout << "Usage: [executable] [-f filename.osm]" << std::endl;
        osm_data_file = "../map.osm";
    }
    
    std::vector<std::byte> osm_data;
 
    if( osm_data.empty() && !osm_data_file.empty() ) {
        std::cout << "Reading OpenStreetMap data from the following file: " <<  osm_data_file << std::endl;
        auto data = ReadFile(osm_data_file);
        if( !data )
            std::cout << "Failed to read." << std::endl;
        else
            osm_data = std::move(*data);
    }
    
    // Input values and validation
    std::cout << "Enter values for start_x, start_y, end_x and end_y in this order using numbers from 0 to 100.\n";
    float start_x = GetIntFromRange(0, 100);
    float start_y = GetIntFromRange(0, 100);;
    float end_x = GetIntFromRange(0, 100);;
    float end_y = GetIntFromRange(0, 100);;

    std::cout << "Start x: " << start_x << std::endl;
    std::cout << "Start y: " << start_y << std::endl;
    std::cout << "End x: " << end_x << std::endl;
    std::cout << "End y: " << end_y << std::endl;

    // Build Model.
    RouteModel model{osm_data};

    // Create RoutePlanner object and perform A* search.
    RoutePlanner route_planner{model, start_x, start_y, end_x, end_y};
    route_planner.AStarSearch();

    std::cout << "Distance: " << route_planner.GetDistance() << " meters. \n";

    // Render results of search.
    Render render{model};

    auto display = io2d::output_surface{400, 400, io2d::format::argb32, io2d::scaling::none, io2d::refresh_style::fixed, 30};
    display.size_change_callback([](io2d::output_surface& surface){
        surface.dimensions(surface.display_dimensions());
    });
    display.draw_callback([&](io2d::output_surface& surface){
        render.Display(surface);
    });
    display.begin_show();
}
