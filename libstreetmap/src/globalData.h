#pragma once

#include <map>
#include <unordered_set>
#include <iostream>
#include "m1.h"
#include "StreetsDatabaseAPI.h"
#include "OSMDatabaseAPI.h"
#include "OSMID.h"
#include "m1Helpers.h"
#include "ezgl/graphics.hpp"
#include "KDTree.h"

class AdditionalData {
    private:
        std::vector<std::vector<StreetSegmentIdx>> segmentsOfStreet;
        void loadSegmentsOfStreet(int num_streets, int num_street_segments);
        void unloadSegmentsOfStreet();

        std::vector<double> travelTimeOfSegment;
        void loadTravelTimeOfSegment(int num_street_segments);
        void unloadTravelTimeOfSegment();

        std::unordered_map<OSMID, const OSMWay*> OSMWayByOSMID;
        void loadOSMWayByOSMID(int num_ways);
        void unloadOSMWayByOSMID();

        std::unordered_map<OSMID, const OSMNode*> OSMNodeByOSMID;
        void loadOSMNodeByOSMID(int num_nodes);
        void unloadOSMNodeByOSMID();

        std::vector<std::vector<StreetSegmentIdx>> segmentsOfIntersection;
        void loadSegmentsOfIntersection(int num_intersections);
        void unloadSegmentsOfIntersection();

        std::multimap<std::string, StreetIdx> streetsLookUpMap;
        void loadStreetsLookUpMap(int num_streets);
        void unloadStreetsLookUpMap();

        std::vector<std::vector<IntersectionIdx>> streetIntersectionsLookUp;
        void loadStreetIntersections(int num_streets);
        void unloadStreetIntersections();
        
        KDTree closestIntersectionLookUp;
        void loadClosestIntersection(int num_intersection);
        void unloadClosestIntersection();

        // The average latitude of the entire loaded map
        double mapAvgLat;
        void loadMapAvgLat(int num_intersections);

        // The width of the loaded map in cartesian coordinates
        double mapWidth;
        void loadMapWidth(double min, double max);

        // min and max x and y of the whole map -- MUST LOAD mapAvgLat FIRST!
        double minX, minY, maxX, maxY;
        void loadMapBounds(int num_intersections);

        // The value at index [street_id] is the length of that street
        std::vector<double> lengthOfStreet;
        void loadLengthOfStreet();
        void unloadLengthOfStreet();
    
    public:
        const std::vector<StreetSegmentIdx>& getSegmentsOfStreet(StreetIdx street_id) const;
        double getTravelTimeOfSegment(StreetSegmentIdx street_segment_id) const;
        const OSMWay* getOSMWayFromOSMID(OSMID id) const;
        const OSMNode* getOSMNodeFromOSMID(OSMID id) const;
        const std::vector<StreetSegmentIdx>& getSegmentsOfIntersection(IntersectionIdx intersection_id) const;
        auto getPrefixLowestBound(std::string prefix) {
            return streetsLookUpMap.lower_bound(prefix);
        }
        auto getEndOfStreetLookUpMap() {
            return streetsLookUpMap.end();
        }
        const std::vector<IntersectionIdx> getStreetIntersections (StreetIdx street_id);
        int getClosestIntersection(LatLon my_position);
        StreetIdx getStreetID(std::string street_name);
        
        double getMapAvgLat() const;
        double getMapWidth() const;
        double getMinX() const;
        double getMaxX() const;
        double getMinY() const;
        double getMaxY() const;
        
        double getLengthOfStreet(StreetIdx street_id) const;

        friend bool loadMap(std::string map_streets_database_filename);
        friend void closeMap();
};

extern AdditionalData GlobalData;


enum ColorScheme {
    LIGHT = 0,
    DARK,
    CONTRAST_LIGHT,
    CONTRAST_DARK
};

// For managing the colors of drawn objects
class ColorHandler {
    private:
        int num_themes;
        ColorScheme current_theme;
        std::string current_theme_name;
        ezgl::color background_color;
        ezgl::color street_color;
        ezgl::color arrow_color;
        ezgl::color selection_color;
        ezgl::color feature_colors[11];
        ezgl::color label_color;
        ezgl::color poi_color;
        ezgl::color text_box_color;

    public:
        // Default constructor initializes using LIGHT theme
        ColorHandler();
        void pickColorScheme(ColorScheme theme);
        int getNumThemes();
        ColorScheme getCurrentTheme();
        std::string getCurrentThemeName();
        ezgl::color getBackgroundColor();
        ezgl::color getStreetColor();
        ezgl::color getArrowColor();
        ezgl::color getSelectionColor();
        ezgl::color getFeatureColor(FeatureType type);
        ezgl::color getLabelColor();
        ezgl::color getPOIColor();
        ezgl::color getTextBoxColor();
};
extern ColorHandler ColorPicker;

/*
// Sorts StreetsDatabaseAPI's entities by their location (in xy)
class ChunkManager {
    private:
        void loadChunkDivisions(int num_divisions);
        std::vector<double> x_divisions; // chunks lie to the right of their x divisions
        std::vector<double> y_divisions; // chunks lie above their y divisions

        // Structure: chunkX<chunkY<vectorOfItems<Items>>>
        std::vector<std::vector<std::vector<FeatureIdx>>> features;
    public:
        std::pair<int, int> findChunk(std::pair<double, double> point);
        
        friend bool loadMap(std::string map_streets_database_filename);
        friend void closeMap();
};
extern ChunkManager Chunks;
*/