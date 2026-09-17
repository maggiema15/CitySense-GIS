#include "m1Helpers.h"
#include <math.h>

//simplifies passed string by removing white_spaces and lowercasing the string, and returning it
std::string simplifyString(std::string original_string)        
{
    std::string simplifiedString;
    simplifiedString.resize(0);
    //iterate through the string, and turn all non-space characters lower-case
    //move all non-space characters into new string
    for (int c = 0; c < original_string.size(); c++)
    {
        if (original_string[c] != ' ') 
        {
            simplifiedString.push_back(std::tolower(original_string[c]));     //push back the lowercased, nonspace character into string vector
        }
    }
    return simplifiedString;
}

// Provided a LatLon value and an average latitude, returns the (x,y) coordinates of the point
std::pair<double, double> LatLonToXY(LatLon lat_lon, double avg_lat) {
    return {lonToX(lat_lon.longitude(), avg_lat), latToY(lat_lon.latitude())};
}
// Provided a longitude and an average latitude, returns an x coordinate
double lonToX(double lon, double avg_lat) {
    return kEarthRadiusInMeters * lon * cos(kDegreeToRadian * avg_lat) * kDegreeToRadian;
}
// Provided a latitude, returns a y coordinate
double latToY(double lat) {
    return kEarthRadiusInMeters * lat * kDegreeToRadian;
}

double yToLat(double y) {
    // Convert meters to degrees latitude
    return y / kEarthRadiusInMeters * (180.0 / M_PI); 
}

double xToLon(double x, double avg_lat) {
    // Convert meters to degrees longitude at avg latitude
    return x / (kEarthRadiusInMeters * cos(avg_lat * M_PI / 180.0)) * (180.0 / M_PI);
}