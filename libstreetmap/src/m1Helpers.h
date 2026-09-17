#pragma once

#include "m1.h"

std::string simplifyString(std::string original_string);
// Provided a LatLon value and an average latitude, returns the (x,y) coordinates of the point
std::pair<double, double> LatLonToXY(LatLon lat_lon, double avg_lat);
// Provided a longitude and an average latitude, returns an x coordinate
double lonToX(double lon, double avg_lat);
// Provided a latitude, returns a y coordinate
double latToY(double lat);
// Provided a x coordinate, returns a longitude
double xToLon(double x, double avg_lat);
// Provided a y coordinae, returns a latitude
double yToLat(double y);