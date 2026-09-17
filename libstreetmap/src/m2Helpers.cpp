#include "m2Helpers.h"
#include <cmath>
#include <unordered_set>
#include <string>

bool intersects(const bounding_box &a, const bounding_box &b) {
    // No overlap if separated on x or y
    if (a.right < b.left) return false;
    if (a.left > b.right) return false;
    if (a.top < b.bottom) return false;
    if (a.bottom> b.top) return false;
    return true;
}

// Precomputes (caches) street label data (position, rotation, text, color, endpoints)
// Drawing code can just iterate street_labels and draw quickly
void build_street_label_cache(std::vector<StreetLabel> &street_labels){
    // Clear old labels
    street_labels.clear();
    int num_segments = getNumStreetSegments();

    // Loop through all the street labels
    for (int seg_id = 0; seg_id < num_segments; seg_id++){
        StreetSegmentInfo info = getStreetSegmentInfo(seg_id);
        std::string name = getStreetName(info.streetID);
        // Skip garbage names
        if (name.empty() || name == "<noname>" || name == "<unknown>"){
            continue;
        }

        // Get endpoints (ignore curve points) in LatLon
        LatLon from = getIntersectionPosition(info.from);
        LatLon to = getIntersectionPosition(info.to);

        //Convert to xy coordinates
        double x1 = lonToX(from.longitude(), GlobalData.getMapAvgLat());
        double y1 = latToY(from.latitude());
        double x2 = lonToX(to.longitude(), GlobalData.getMapAvgLat());
        double y2 = latToY(to.latitude());

        // Vector direction from point 1 to point 2 in XY space
        double dx = x2 - x1;
        double dy = y2 - y1;

        // Compute the rotation angle
        double angle_deg = std::atan2(dy, dx) * 180.0 / M_PI;

        // Prevent upside down labelling
        // Keep the label rotation within [-90, 90] degrees for readability
        if (angle_deg > 90.0){
            angle_deg -= 180.0;
        }
        if (angle_deg < -90.0){
            angle_deg += 180.0;
        }

        //Display at midpoint
        double mid_x = (x1 + x2) / 2.0;
        double mid_y = (y1 + y2) / 2.0;

        // Store label information
        StreetLabel label;
        label.pos = {mid_x, mid_y};
        label.angle_deg = angle_deg;
        label.text = name;
        label.from_xy = {x1, y1};
        label.to_xy = {x2, y2};
        label.street_id = info.streetID;
        label.speed_limit = info.speedLimit;

        //Add to cache
        street_labels.push_back(label);
    }
}

// Precomputes (caches) POI label data (position, text, color, type)
// Drawing code can just iterate POI_labels and draw quickly
void build_poi_label_cache(std::vector<POILabel> &poi_labels){
    // Clear old labels
    poi_labels.clear();

    int num_pois = getNumPointsOfInterest();

    // Reserve capacity up front to reduce reallocations as we push_back
    poi_labels.reserve(num_pois);

    // Loop through all the POI labels
    for (int poi_id = 0; poi_id < num_pois; poi_id++){
        std::string name = getPOIName(poi_id);

        // Skip garbage names
        if (name.empty() || name == "<noname>" || name == "<unknown>"){
            continue;
        }

        // POI location in LatLon
        LatLon location = getPOIPosition(poi_id);

        // Convert LatLon to XY
        double x = lonToX(location.longitude(), GlobalData.getMapAvgLat());
        double y = latToY(location.latitude());

        // Store label information
        POILabel label;
        label.pos = {x, y};
        label.text = name;
        label.poi_type = getPOIType(poi_id);

        poi_labels.push_back(label);
    }
}

// Provided the ezgl renderer and a collection of points, 
// returns true if the resulting bounding box intersects the current visible world.
bool isObjectVisible(ezgl::renderer *g, std::vector<ezgl::point2d> points) {
    double left = points[0].x, right = points[0].x;
    double bottom = points[0].y, top = points[0].y;
    for (int i = 0; i < points.size(); i++) {
        left = std::min(left, points[i].x);
        right = std::max(right, points[i].x);
        bottom = std::min(bottom, points[i].y);
        top = std::max(top, points[i].y);
    }
    ezgl::rectangle current = g->get_visible_world();
    if (left > current.right() || right < current.left() || 
        bottom > current.top() || top < current.bottom()) { return false; }
    else { return true; }
}

//precomputes the intersection labels on the map
//stores all of the precomputed data in a vector of Intersection Labels that can be accessed 
void build_intersection_label_cache(std::vector<IntersectionLabel> &intersection_labels){
   intersection_labels.clear();

   int num_intersections = getNumIntersections();
   intersection_labels.reserve(num_intersections);

   for (int intersection_id = 0; intersection_id < num_intersections; intersection_id++){
      LatLon location = getIntersectionPosition(intersection_id);

      double x = lonToX(location.longitude(), GlobalData.getMapAvgLat());
      double y = latToY(location.latitude());

      IntersectionLabel label;
      label.pos = {x, y};
      label.isSelected = false;
      label.id = intersection_id;

      intersection_labels.push_back(label);
   }
}

// Provided the ezgl renderer and a point, returns true if the point is within the current visible world.
bool isPointVisible(ezgl::renderer *g, double x, double y) {
    ezgl::rectangle visible_world = g->get_visible_world();
    return (x <= visible_world.right() && x >= visible_world.left() &&
            y <= visible_world.top() && y >= visible_world.bottom());
}

// Provided the ezgl renderer and a street segment id, draws the segment according to zoom rules
/*
Zoom tier 1: roads >= 20m/s
Zoom tier 2: roads >= 13m/s
Zoom tier 3: all roads
Zoom tier 4: all roads and thicker
Zoom tier 5: even thicker!
*/
void drawStreetSegment(ezgl::renderer *g, StreetSegmentIdx segment, ezgl::color color) {
    StreetSegmentInfo segment_info = getStreetSegmentInfo(segment);
    double street_length = GlobalData.getLengthOfStreet(segment_info.streetID);
    // Determine zoom level
    double current_width = g->get_visible_world().width();
    double street_width;
    if (current_width > 10000) {               // Tier 1
        if (getStreetName(segment_info.streetID) == "<unknown>") {
            return;
        } else if (segment_info.speedLimit >= 20) {
            street_width = 2;
        } else if (street_length > 5000) {
            street_width = 1;
        } else { return; }
    } else if (current_width > 3000) {         // Tier 2
        if (getStreetName(segment_info.streetID) == "<unknown>") {
            return;
        } else if (segment_info.speedLimit >= 20) {
            street_width = 2;
        } else if (segment_info.speedLimit >= 11 || street_length > 5000) {
            street_width = 1;
        } else { return; }
    } else if (current_width > 1500){          // Tier 3
        if (getStreetName(segment_info.streetID) == "<unknown>") {
            street_width = 1;
        } else if (segment_info.speedLimit >= 20) {
            street_width = 4;
        } else if (street_length > 5000) {
            street_width = 3;
        } else if (segment_info.speedLimit >= 11) {
            street_width = 2;
        } else {
            street_width = 2;
        }
    } else if (current_width > 400) {          // Tier 4
        if (getStreetName(segment_info.streetID) == "<unknown>") {
            street_width = 2;
        } else if (segment_info.speedLimit >= 20) {
            street_width = 8;
        } else if (street_length > 5000) {
            street_width = 6;
        } else if (segment_info.speedLimit >= 11) {
            street_width = 4;
        } else {
            street_width = 3;
        }
    } else {                                   // Tier 5
        if (getStreetName(segment_info.streetID) == "<unknown>") {
            street_width = 3;
        } else if (segment_info.speedLimit >= 20 || street_length > 5000) {
            street_width = 16;
        } else if (street_length > 5000) {
            street_width = 12;
        } else if (segment_info.speedLimit >= 11) {
            street_width = 8;
        } else {
            street_width = 4;
        }
    }
        
    std::pair<double, double> from = LatLonToXY(getIntersectionPosition(segment_info.from), 
                                                GlobalData.getMapAvgLat());
    std::pair<double, double> to = LatLonToXY(getIntersectionPosition(segment_info.to), 
                                              GlobalData.getMapAvgLat());

    std::vector<ezgl::point2d> points;
    points.push_back({from.first, from.second});
    if (segment_info.numCurvePoints > 0) {
        for (int i = 0; i < segment_info.numCurvePoints; i++) {
            std::pair<double, double> point = LatLonToXY(getStreetSegmentCurvePoint(segment, i), 
                                                            GlobalData.getMapAvgLat());
            points.push_back({point.first, point.second});
        }
    }
    points.push_back({to.first, to.second});

    g->set_line_cap(ezgl::line_cap::round);
    g->set_line_width(street_width);
    g->set_color(color);
    for (int i = 1; i < points.size(); i++) {
        g->draw_line(points[i-1], points[i]);
    }
    // Draw arrows
    if (segment_info.oneWay && current_width < 400) {
        g->set_line_width(street_width / 3);
        g->set_line_cap(ezgl::line_cap::butt);
        g->set_color(ColorPicker.getArrowColor());
        // For curved segment
        if (points.size() > 2) {
            double curve_point_density = segment_info.numCurvePoints / findStreetSegmentLength(segment);
            for (int i = 1; i < points.size(); i++) {
                if (!(i % ((int)(curve_point_density * 150) + 1))) { // More points get arrows if there are fewer points
                    double angle = atan2(points[i].y - points[i-1].y, points[i].x - points[i-1].x);
                    drawArrow(g, points[i-1], angle, street_width);
                }
            }
        // For straight segment
        } else {
            double angle = atan2(points[1].y - points[0].y, points[1].x - points[0].x);
            double length = findStreetSegmentLength(segment);
            int num_arrows = 0.015 * length;
            double dx = (length / num_arrows) * cos(angle);
            double dy = (length / num_arrows) * sin(angle);
            for (int i = 1; i < num_arrows; i++) {
                drawArrow(g, {points[0].x + (i * dx), points[0].y + (i * dy)}, angle, street_width);
            }
        }
        g->set_line_cap(ezgl::line_cap::round);
    }
}

// Provided the ezgl renderer, a point, and an angle, draws an arrow
void drawArrow(ezgl::renderer *g, ezgl::point2d center, double angle, int street_width) {
    double scale_factor = street_width / 3.0;
    double dx = scale_factor * cos(angle);
    double dy = scale_factor * sin(angle);
    std::vector<ezgl::point2d> points = {{center.x + dx, center.y + dy}, 
                                         {center.x - (dy / (0.6 * scale_factor)), center.y + (dx / (0.6 * scale_factor))},
                                         {center.x + (dy / (0.6 * scale_factor)), center.y - (dx / (0.6 * scale_factor))}};
    g->fill_poly(points);
    g->draw_line(center, {center.x - dx, center.y - dy});
}

void drawFeature(ezgl::renderer *g, FeatureIdx feature) {
    g->set_color(ColorPicker.getFeatureColor(getFeatureType(feature)));

   // Construct vector of points
    std::vector<ezgl::point2d> points;
    for (int i = 0; i < getNumFeaturePoints(feature); i++) {
      std::pair<double, double> point = LatLonToXY(getFeaturePoint(feature, i), GlobalData.getMapAvgLat());
      points.push_back({point.first, point.second});
    }

   // Closed polygon or line?
    if (getFeaturePoint(feature, 0) == getFeaturePoint(feature, getNumFeaturePoints(feature) - 1) && points.size() > 2) {
      g->fill_poly(points);
    } else {
        g->set_line_cap(ezgl::line_cap::round);
        if (getFeatureType(feature) == RIVER) { g->set_line_width(2); }
        else if (getFeatureType(feature) == STREAM && g->get_visible_world().width() > 10000) { return; }
        else { g->set_line_width(2); } 
        for (int i = 1; i < points.size(); i++) {
            g->draw_line(points[i - 1], points[i]);
        }
    g->set_line_cap(ezgl::line_cap::butt);
    }  
}

// Returns true if the segment's bounding box intersects the view rectangle
static bool segment_intersects_view(const ezgl::rectangle &view,
                                    const ezgl::point2d &a,
                                    const ezgl::point2d &b) {
    // Compute bounding box of the segment
    // Smallest rectangle that contains the line segment
    double left = std::min(a.x, b.x);
    double right = std::max(a.x, b.x);
    double bottom = std::min(a.y, b.y);
    double top = std::max(a.y, b.y);

    // Check if the segment box is completely out of the view
    if (right < view.left() || left > view.right()){
        return false;
    }
    if (top < view.bottom() || bottom > view.top()){
        return false;
    }
    return true;
}

// Clamp a point into the view rectangle
// If point is outside view, snap it to the nearest edge
static ezgl::point2d clamp_to_view(const ezgl::rectangle &view, ezgl::point2d p) {
    // If x is left of view → move it to left boundary
    if (p.x < view.left()) p.x = view.left();
    // If x is right of view → move it to right boundary
    if (p.x > view.right()) p.x = view.right();
    // If y is below view → move it to bottom boundary
    if (p.y < view.bottom()) p.y = view.bottom();
    // If y is above view → move it to top boundary
    if (p.y > view.top()) p.y = view.top();
    return p;
}

// Helper that tries to place one street label with overlap checks.
// Returns true if the label was drawn, false if skipped/collided.
static bool try_draw_street_label(ezgl::renderer *g,
                                 const ezgl::rectangle &view,
                                 const StreetLabel &label,
                                 const std::string &name,
                                 std::vector<bounding_box> &occupied_boxes,
                                 std::unordered_set<int> &labeled_streets,
                                 double world_per_px,
                                 double char_px_w,
                                 double text_px_h,
                                 double padding_px) {

    // If already labeled this street ID, skip
    if (labeled_streets.find(label.street_id) != labeled_streets.end()) {
        return false;
    }

    // Skip garbage names
    if (name.empty() || name == "<noname>" || name == "<unknown>") {
        return false;
    }

    // Skip if not in view
    if (!segment_intersects_view(view, label.from_xy, label.to_xy)) {
        return false;
    }

    // Compute midpoint of the visible frame
    ezgl::point2d a = clamp_to_view(view, label.from_xy);
    ezgl::point2d b = clamp_to_view(view, label.to_xy);
    ezgl::point2d visible_mid{ (a.x + b.x) / 2.0, (a.y + b.y) / 2.0 };

    if (!view.contains(visible_mid)) {
        return false;
    }

    // Visible segment length (prevents cramped labels)
    double seg_len = std::hypot(b.x - a.x, b.y - a.y);

    // Estimate unrotated text box size in world units
    double W = (char_px_w * (double)name.length()) * world_per_px;
    double H = (text_px_h) * world_per_px;

    // Add padding on all sides
    double pad = padding_px * world_per_px;
    W += 2.0 * pad;
    H += 2.0 * pad;

    // If visible segment length is smaller than label width, don’t draw
    if (seg_len < 1.1 * W) {
        return false;
    }

    // Inflate to bounding_box that covers rotated rectangle
    double theta = label.angle_deg * M_PI / 180.0;
    double c = std::fabs(std::cos(theta));
    double s = std::fabs(std::sin(theta));

    double bounding_box_w = W * c + H * s;
    double bounding_box_h = W * s + H * c;

    bounding_box box;
    box.left = visible_mid.x - bounding_box_w / 2.0;
    box.right = visible_mid.x + bounding_box_w / 2.0;
    box.bottom = visible_mid.y - bounding_box_h / 2.0;
    box.top = visible_mid.y + bounding_box_h / 2.0;

    // Collision test against previously placed labels
    for (size_t j = 0; j < occupied_boxes.size(); j++) { 
        const bounding_box &placed = occupied_boxes[j];
        if (intersects(box, placed)) {
            return false;
        }
    }

    // Draw a background on text in high contrast modes
    if (ColorPicker.getCurrentTheme() >= CONTRAST_LIGHT) {
        g->set_color(ColorPicker.getTextBoxColor(), 200);
        g->fill_rectangle({box.left, box.bottom}, {box.right, box.top});
    }
    // Draw
    g->set_color(ColorPicker.getLabelColor(), 255);
    g->set_text_rotation(label.angle_deg);
    g->draw_text(visible_mid, name);
    g->set_text_rotation(0);

    // Record placement
    occupied_boxes.push_back(box);
    labeled_streets.insert(label.street_id);
    return true;
}

void draw_street_labels(ezgl::renderer *g,
                        const std::vector<StreetLabel> &street_labels,
                        std::vector<bounding_box> &occupied_boxes){

    // Current visible world rectangle (in world coords) and its width
    ezgl::rectangle view = g->get_visible_world();
    g->format_font("Sans", ezgl::font_slant::normal, ezgl::font_weight::bold);
    double view_width = view.width();

    // Only show labels when zoomed in enough
    if (view_width > 40000.0) {
        return;
    }

    // Zoom based density control
    int stride; // Only examine every stride-th label in the cache
    int max_labels;

    if (view_width > 16000.0){ 
        stride = 18; 
        max_labels = 200; 
    }
    else if (view_width > 12000.0) { 
        stride = 12; 
        max_labels = 260; 
    }
    else if (view_width > 8000.0) { 
        stride = 8; 
        max_labels = 340; 
    }
    else if (view_width > 4000.0) { 
        stride = 5;  
        max_labels = 460; 
    }
    else { 
        stride = 3;  
        max_labels = 650; 
    }

    int drawn = 0; // Label drawn tally

    // Keep track of already placed label rectangles (world coord)
    occupied_boxes.reserve(occupied_boxes.size() + max_labels);

    // Prevent the same street from being labeled multiple times
    std::unordered_set<int> labeled_streets;

    //----- Overlap prevention logic -----
    const double assumed_canvas_pixel_width = 1000.0;

    // Rough font metrics in pixels
    const double char_px_w = 7.0;
    const double text_px_h = 12.0;
    const double padding_px = 6.0;

    const double world_per_px = view_width / assumed_canvas_pixel_width;

    // Speed tiers (m/s)
    const double MAJOR_SPEED  = 20.0;  // m/s (~80 km/h)
    const double MEDIUM_SPEED = 13.0;  // m/s (~50 km/h)

    // One-pass priority with tier budgets
    // Allocate most label slots to major roads, then medium, then minor
    int major_cap = (int)(0.60 * max_labels); // 60%
    int medium_cap = (int)(0.30 * max_labels); // 30%
    int minor_cap = max_labels - major_cap - medium_cap;

    int major_drawn = 0;
    int medium_drawn = 0;
    int minor_drawn = 0;

    for (size_t i = 0; i < street_labels.size(); i += stride){
        const StreetLabel &label = street_labels[i];
        const std::string &name = label.text;

        // ---- Priority logic ----
        // Decide tier
        // Skip if the tier already filled its quota
        // Attempt placement with overlap rules
        // Count successful draws
        // Stop when max labels reached

        // Determine speed tier
        int tier = 0;
        if (label.speed_limit >= MAJOR_SPEED){
            tier = 2;
        }
        else if (label.speed_limit >= MEDIUM_SPEED){
            tier = 1;
        }
        else {
            tier = 0;
        }

        // Enforce tier
        if (tier == 2 && major_drawn >= major_cap){
            continue;
        }
        if (tier == 1 && medium_drawn >= medium_cap){
            continue;
        }
        if (tier == 0 && minor_drawn >= minor_cap){
            continue;
        }

        // Try to draw using the same overlap logic as before
        bool drew = try_draw_street_label(g, view, label, name,
                                         occupied_boxes, labeled_streets,
                                         world_per_px, char_px_w, text_px_h, padding_px);

        if (!drew){
            continue;
        }

        // Update counters
        if (tier == 2){
            major_drawn++;
        }
        else if (tier == 1){
            medium_drawn++;
        }
        else{
            minor_drawn++;
        }

        if (++drawn >= max_labels) {
            break;
        }
    }
}

void draw_poi_labels(ezgl::renderer *g,
                     const std::vector<POILabel> &poi_labels,
                     std::vector<bounding_box> &occupied_boxes){
    ezgl::rectangle view = g->get_visible_world();
    g->format_font("Sans", ezgl::font_slant::normal, ezgl::font_weight::bold);
    double view_width = view.width();

    // Only show POIs when zoomed in enough
    if (view_width > 7000.0){
        return;
    }

    // Zoom-based density control
    int stride;
    int max_labels;

    if (view_width > 16000.0) { 
        stride = 85; 
        max_labels = 140; 
        }
    else if (view_width > 12000.0) { 
        stride = 70; 
        max_labels = 170; 
        }
    else if (view_width > 8000.0) { 
        stride = 55; 
        max_labels = 220; 
        }
    else if (view_width > 4000.0) { 
        stride = 35;  
        max_labels = 300; 
        }
    else { 
        stride = 25;  
        max_labels = 420; 
        }
    
    int drawn = 0;

    // Track placed POI label rectangles (WORLD coords)
    occupied_boxes.reserve(occupied_boxes.size() + max_labels);
    std::unordered_set<std::string> labeled_names;

    const double assumed_canvas_pixel_width = 1000.0;
    const double char_px_w = 7.0;
    const double text_px_h = 12.0;
    const double padding_px = 6.0;

    const double world_per_px = view_width / assumed_canvas_pixel_width;

    for (size_t i = 0; i < poi_labels.size(); i += stride){
        const POILabel &label = poi_labels[i];

        if (!view.contains(label.pos)) {
            continue;
        }

        const std::string &name = label.text;
        if (name.empty() || name == "<noname>" || name == "<unknown>") {
            continue;
        }  

        // Filtering
        if (!poi_type_matches_filter(label.poi_type)) {
            continue;
        }

        // Optional de-dupe by name (comment out if you want repeats)
        if (labeled_names.find(name) != labeled_names.end()) {
            continue;
        }

        // Estimate text box size in WORLD units (no rotation for POIs)
        double W = (char_px_w * (double)name.length()) * world_per_px;
        double H = (text_px_h) * world_per_px;

        double pad = padding_px * world_per_px;
        W += 2.0 * pad;
        H += 2.0 * pad;

        // Build bounding_box around the midpoint
        bounding_box box;
        box.left = label.pos.x - W / 2.0;
        box.right = label.pos.x + W / 2.0;
        box.bottom = label.pos.y - H / 2.0;
        box.top = label.pos.y + H / 2.0;

        // Collision test
        bool collides = false;
        for (size_t j = 0; j < occupied_boxes.size(); j++) {
            const bounding_box &placed = occupied_boxes[j];
            if (intersects(box, placed)) {
                collides = true;
                break;
            }
        }
        if (collides) {
            continue;
        }

        // Draw a background on text in high contrast modes
        if (ColorPicker.getCurrentTheme() >= CONTRAST_LIGHT) {
            g->set_color(ColorPicker.getTextBoxColor(), 200);
            g->fill_rectangle({box.left, box.bottom}, {box.right, box.top});
        }
        
        g->set_color(ColorPicker.getPOIColor(), 255);
        g->draw_text(poi_labels[i].pos, poi_labels[i].text);

        occupied_boxes.push_back(box);
        labeled_names.insert(name);

        if (++drawn >= max_labels){
            break;
        }
    }
}

//draws the intersection labels on the map when selected by the user

void draw_intersection_labels(ezgl::renderer *g, std::vector<IntersectionLabel> &intersection_labels) {
   ezgl::rectangle view = g->get_visible_world();

    for (size_t i = 0; i < intersection_labels.size(); i ++){
        if (!view.contains(intersection_labels[i].pos)){
            continue;
        }

        //only draw labels when selected
        if (intersection_labels[i].isSelected == true) {
            g->set_color(ColorPicker.getSelectionColor());
            g->fill_arc(intersection_labels[i].pos, 5, 0, 360);
        }
    }
}

//function to clear (erase) all of the visible intersection labels
void clear_intersection_labels(std::vector<IntersectionLabel> &intersection_labels) {
    for (int itn = 0; itn < intersection_labels.size(); itn++) {
        if (intersection_labels[itn].isSelected) intersection_labels[itn].isSelected = false;
    }
}

//moves camera to some xy position on the map
void move_camera(ezgl::application *app, double x, double y, double zoom) {
   ezgl::canvas *canvas = app->get_canvas("MainCanvas");
   ezgl::camera &camera = canvas->get_camera();

   double width = zoom * 1.29808103;
   double height = zoom;

   ezgl::rectangle newView({x - width/2, y - height/2}, {x+width/2, y+height/2});

   camera.set_world(newView);
}