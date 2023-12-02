#ifndef DCC067_ACO_ACO_H
#define DCC067_ACO_ACO_H

#include "Graph.h"
#include <queue>
#include <limits>
#include <valarray>
#include <iostream>

typedef struct {
    vector<int> path;
    int charge;
    int battery;
} Vehicle;

typedef struct {
    vector <Vehicle> vehicles;
    vector<bool> visited;
    double solution_value;
} Ant;

void initializeAnts(Graph &g, vector <Ant> &ants, int n);

void initializeParameters(vector <Ant> &ants, Graph &g, float pheromone);

Edge *selectNextClient(Ant &ant, Vehicle &vehicle, Graph &g, float alpha, float beta);
Edge *closestHotel(Graph &g, Node *current_node);

void aco(Graph &g, int cycles, float evaporation, float alpha, float beta);

bool isContainedListCandidates(Vehicle &vehicle, int id_node);

#endif //DCC067_ACO_ACO_H
