#include "route_planner.h"
#include <algorithm>

RoutePlanner::RoutePlanner(RouteModel &model, float start_x, float start_y, float end_x, float end_y): m_Model(model) {
    // Convert inputs to percentage:
    start_x *= 0.01;
    start_y *= 0.01;
    end_x *= 0.01;
    end_y *= 0.01;

    std::cout << "Start: (" << start_x << ", " << start_y << ").\n";
    std::cout << "End: (" << end_x << ", " << end_y << ").\n";
    
    start_node = &m_Model.FindClosestNode(start_x, start_y);
    end_node = &m_Model.FindClosestNode(end_x, end_y);
}

float RoutePlanner::CalculateHValue(RouteModel::Node const *node) {
    return node->distance(*end_node);
}

void RoutePlanner::AddNeighbors(RouteModel::Node *current_node) {
    current_node->FindNeighbors();
    for (RouteModel::Node* node : current_node->neighbors) {
        node->parent = current_node;
        node->h_value = CalculateHValue(node);
        node->g_value = current_node->g_value + current_node->distance(*node);
        node->visited = true;
        open_list.emplace_back(node);
    }
}

RouteModel::Node *RoutePlanner::NextNode() {
    std::sort(open_list.begin(), open_list.end(), [] (const RouteModel::Node* left, const RouteModel::Node* right) {
        return left->h_value + left->g_value < right->h_value + right->g_value;
    });

    RouteModel::Node* lowestNode = open_list.front();
    open_list.erase(open_list.begin());

    return lowestNode;
}

std::vector<RouteModel::Node> RoutePlanner::ConstructFinalPath(RouteModel::Node *current_node) {
    // Create path_found vector
    distance = 0.0f;
    std::vector<RouteModel::Node> path_found;

    while (current_node->parent != nullptr) {
        path_found.emplace_back(*current_node);
        distance += current_node->distance(*current_node->parent);
        current_node = current_node->parent;
    }
    path_found.emplace_back(*current_node);

    std::reverse(path_found.begin(), path_found.end());

    distance *= m_Model.MetricScale(); // Multiply the distance by the scale of the map to get meters.
    return path_found;
}


// TODO 7: Write the A* Search algorithm here.
// Tips:
// - Use the AddNeighbors method to add all of the neighbors of the current node to the open_list.
// - Use the NextNode() method to sort the open_list and return the next node.
// - When the search has reached the end_node, use the ConstructFinalPath method to return the final path that was found.
// - Store the final path in the m_Model.path attribute before the method exits. This path will then be displayed on the map tile.

void RoutePlanner::AStarSearch() {
    this->start_node->visited = true;
    this->open_list.emplace_back(this->start_node);
    
    RouteModel::Node* current_node = nullptr;
    while (!open_list.empty()) {
        current_node = this->NextNode();
        if (current_node->distance(*this->end_node) == 0) { // reach the end_node
            m_Model.path = this->ConstructFinalPath(current_node);
            return; // exit the A* search
        } else { // continue A* search
            AddNeighbors(current_node);
        }
    }
}
