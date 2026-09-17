/*
header file for the implementation of a KDTree data structure
this datastructure is used for finding closest coordinates to a point
this implemention is mainly used for FindClosestIntersection function for m1
this .hcc and the corresponding .cpp codes were obtained from https://www.geeksforgeeks.org/cpp/kd-trees-in-cpp/
with modifications for our purpose
*/
#pragma once
//#define _USE_MATH_DEFINES
#include <array>
#include "m1.h"
#include "StreetsDatabaseAPI.h"
#include "OSMDatabaseAPI.h"
#include "OSMID.h"
//#include "m1Helpers.h"
#include <cmath>

struct Node {
        std::array<double, 2> point;
        int id;
        Node* left;
        Node* right;
        Node(const std::array<double, 2>& pt, const int& new_id) {
            point  = pt;
            id = new_id;
            left = nullptr;
            right = nullptr;
        }
};

class KDTree {
    private:
        struct Node {
            std::array<double, 2> point;
            int id;
            Node* left;
            Node* right;
            Node(const std::array<double, 2>& pt, const int& new_id) {
                point  = pt;
                id = new_id;
                left = nullptr;
                right = nullptr;
            }
        };
        Node* root;

        Node* InsertHelper(Node* node, const std::array<double, 2>& point, const int& id, int depth);
        bool SearchHelper(Node* node, const std::array<double, 2>& point, const int& id, int depth) const;
        void FindClosestHelper(Node* node, const std::array<double, 2>& target, int depth, Node*& best, double& bestDist)const;
        void FreeTree(Node* node);
        void PrintHelper(Node* node, int depth);

    public:
        KDTree();
        ~KDTree();
        void Clear();
        void Insert(const std::array<double, 2>& point, const int& id);
        bool Search(const std::array<double, 2>& point, const int& id)const ;
        int FindClosest(const std::array<double, 2>& point) const;
        void Print();
};