/*
This is the source file for the KDtree data structure
has the function implementations of kdtree
*/

#include "KDTree.h"

KDTree::Node* KDTree::InsertHelper(KDTree::Node* node, const std::array<double, 2>& point, const int& id, int depth) {
    if (node == nullptr) return new KDTree::Node(point, id);
    int axis = depth % 2;
    
    if (point[axis] < node->point[axis])
        node->left = InsertHelper(node->left, point, id, depth+1);
    else    
        node->right = InsertHelper(node->right, point, id, depth+1);

    return node;
}

bool KDTree::SearchHelper(KDTree::Node* node, const std::array<double, 2>& point, const int& id, int depth) const{
    if (node == nullptr) return false;

    if (node->id == id) return true;

    int axis = depth % 2;

    if (point[axis] < node->point[axis])
        return SearchHelper(node->left, point, id, depth+1);
    else 
        return SearchHelper(node->right, point, id, depth+1);
}

void KDTree::FindClosestHelper(KDTree::Node* node, const std::array<double, 2>& target, int depth, KDTree::Node*& best, double& bestDist) const{
    if (node == nullptr) return;

    std::pair<LatLon, LatLon> AB = {LatLon(node->point[0], node->point[1]), LatLon(target[0], target[1])};
    double distanceBetween = findDistanceBetweenTwoPoints(AB);

    if (distanceBetween < bestDist) {
        bestDist = distanceBetween;
        best = node;
    }

    int axis = depth % 2;

    Node* nextBranch;
    Node* otherBranch;

    if (target[axis] < node->point[axis]) {
        nextBranch = node->left;
        otherBranch = node->right;
    } else {
        nextBranch = node->right;
        otherBranch = node->left;
    }
    double minAxisDist;

    if (axis == 0)
        minAxisDist = 111320.0 * std::abs(target[0] - node->point[0]);
    else {
        double lonDiff = std::abs(target[1]-node->point[1]);
        lonDiff = std::min(lonDiff, 360.0 - lonDiff);

        double latRad = target[0] * M_PI / 180.0;
        minAxisDist = 111320.0 * cos(latRad) * lonDiff;
    }

    FindClosestHelper(nextBranch, target, depth + 1, best, bestDist);
    
    if (minAxisDist < bestDist) {
        FindClosestHelper(otherBranch, target, depth+1, best, bestDist);
    }    
}

void KDTree::FreeTree(KDTree::Node* node) {
    if (!node) return;
    FreeTree(node->left);
    FreeTree(node->right);
    delete node;
}

void KDTree::PrintHelper(Node* node, int depth) {
    if (node == nullptr) return;
    for (int i = 0; i < depth; i++) std::cout << " ";
    std::cout << "(";
    for (size_t i =0; i < 2; i++) {
        std::cout << node->point[i];
        if (i < 1) std::cout << ", ";
    }

    std::cout << ", " << node->id << ")" << std::endl;

    PrintHelper(node->left, depth+1);
    PrintHelper(node->right, depth+1);
}

KDTree::KDTree() {
    root = nullptr;
}

KDTree::~KDTree() {
    FreeTree(root);
}

void KDTree::Clear() {
    FreeTree(root);
    root = nullptr;
}
void KDTree::Insert(const std::array<double, 2>& point, const int& id) {
    root = InsertHelper(root, point, id, 0);
}

bool KDTree::Search(const std::array<double, 2>& point, const int& id) const {
    return SearchHelper(root, point, id, 0);
}

int KDTree::FindClosest(const std::array<double, 2>& point) const{
    if (root == nullptr) return -1;

    KDTree::Node* best = nullptr;
    double bestDist = 1e16;
    FindClosestHelper(root, point, 0, best, bestDist);
    
    return best->id;
}

void KDTree::Print() {
    PrintHelper(root, 0);
}