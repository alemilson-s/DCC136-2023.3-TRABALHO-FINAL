#include "Edge.h"
#include <iostream>

using namespace std;

Edge::Edge(int target_id) {

    this->target_id = target_id;
    this->next_edge = nullptr;
    this->weight = -1;
    this->pheromone = -1;
}

Edge::~Edge() {

    if (this->next_edge != nullptr) {
        delete this->next_edge;
        this->next_edge = nullptr;
    }

}

int Edge::getTargetId() {

    return this->target_id;

}

Edge *Edge::getNextEdge() {

    return this->next_edge;

}

double Edge::getWeight() {

    return this->weight;

}

void Edge::setNextEdge(Edge *edge) {

    this->next_edge = edge;

}

void Edge::setWeight(double weight) {

    this->weight = weight;

}

double Edge::getPheromone() const {
    return pheromone;
}

void Edge::setPheromone(double pheromone) {
    Edge::pheromone = pheromone;
}
