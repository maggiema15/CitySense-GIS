#pragma once

#include "globalData.h"
#include "m1.h"
#include "m1Helpers.h"
#include "ezgl/application.hpp"
#include "ezgl/graphics.hpp"

struct StreetLabel{
   ezgl::point2d pos; //where to draw text
   double angle_deg; //rotation angle
   std::string text; //street name
   ezgl::point2d from_xy;
   ezgl::point2d to_xy;
   double speed_limit;
   int street_id;
};

struct POILabel{
   ezgl::point2d pos; // where to draw text (XY)
   std::string text; // POI name
   std::string poi_type; //category
};

//Label collision helpers
struct bounding_box {
    double left;
    double right;
    double bottom;
    double top;
};

bool intersects(const bounding_box &a, const bounding_box &b);

bool poi_type_matches_filter(std::string const &poi_type);

//label of intersection is a small circle that changes color when selected
struct IntersectionLabel{
   ezgl::point2d pos;   //where the intersection label is
   bool isSelected;     //state of button, checks if the user has selected the intersection or not
   IntersectionIdx id;  //id of intersection represented by the label
};

void build_poi_label_cache(std::vector<POILabel> &poi_labels);
void draw_poi_labels(ezgl::renderer *g,
                     const std::vector<POILabel> &poi_labels,
                     std::vector<bounding_box> &occupied_boxes);

void build_street_label_cache(std::vector<StreetLabel> &street_labels);

// Provided the ezgl renderer and a collection of points, 
// returns true if the resulting bounding box intersects the current visible world.
bool isObjectVisible(ezgl::renderer *g, std::vector<ezgl::point2d> points);
void build_intersection_label_cache(std::vector<IntersectionLabel> &intersection_labels);
void draw_intersection_labels(ezgl::renderer *g, std::vector<IntersectionLabel> &intersection_labels);
void clear_intersection_labels(std::vector<IntersectionLabel> &intersection_labels);

// Provided the ezgl renderer and a point, returns true if the point is within the current visible world.
bool isPointVisible(ezgl::renderer *g, double x, double y);
// Provided the ezgl renderer and a street segment id, draws the segment
// Will not draw if endpoint intersections are out of bounds
// Thickness is based on speed limit
// TODO: communicate one-way ness
void drawStreetSegment(ezgl::renderer *g, StreetSegmentIdx segment, ezgl::color color);

// Provided the ezgl renderer, a point, and an angle, draws an arrow
void drawArrow(ezgl::renderer *g, ezgl::point2d center, double angle, int street_width);

// Provided the ezgl renderer and a feature id, draws the feature
// Will not draw if every point is out of bounds
void drawFeature(ezgl::renderer *g, FeatureIdx feature);

void draw_street_labels(ezgl::renderer *g,
                        const std::vector<StreetLabel> &street_labels,
                        std::vector<bounding_box> &occupied_boxes);

//moves the camera to a position xy on the screen
//zooms in on the selected point for better view
void move_camera(ezgl::application *app, double x, double y, double zoom);

