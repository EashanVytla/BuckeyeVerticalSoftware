#include <iostream>
#include "localization.h"

int main() {
    // Example usage
    double current_lat = 40.097977;  // Current latitude
    double current_lon = -83.198335; // Current longitude
    double heading = 225.0;           // Heading in degrees (0 = North, 90 = East, 180 = South, 270 = West)
    double offset_x = 0.0;        // X offset in meters (positive = Right, negative = Left)
    double offset_y = 20.0;        // Y offset in meters (positive = Up, negative = Down)

    std::cout << std::fixed << std::setprecision(16);

    std::pair<double, double> new_coords = offset_to_latlon(current_lat, current_lon, heading, offset_x, offset_y);
    double new_lat = new_coords.first;
    double new_lon = new_coords.second;

    std::cout << "New latitude: " << new_lat << std::endl;
    std::cout << "New longitude: " << new_lon << std::endl;

    return 0;
}