#ifndef DCC067_ACO_ACO_H
#define DCC067_ACO_ACO_H

#include "Graph.h"
#include <queue>
#include <limits>
#include <valarray>
#include <iostream>

typedef struct {
    vector<int> path;
    float charge;
    float battery;
} Vehicle;

typedef struct {
    vector <Vehicle> vehicles;
    vector<bool> visited;
    double solution_value;
} Ant;

void initializeAnts(Graph &g, vector <Ant> &ants, int n);

void initializeParameters(vector <Ant> &ants, Graph &g);

Edge *selectNextClient(Ant &ant, Vehicle &vehicle, Graph &g, float alpha, float beta, float q0);

Edge *closestHotel(Graph &g, Node *current_node);

void aco(Graph &g, int cycles, float evaporation, float alpha, float beta);

bool isContainedListCandidates(Vehicle &vehicle, int id_node);

bool canVisitClient(Node *node, Vehicle &vehicle, Graph &g, int current_node);

Edge *closestStop(Graph &g, Node *current_node);

bool meetsDemand(Vehicle &vehicle, Graph &g,Ant &ant);

bool canVisitStation(Node *node, Vehicle &vehicle, Graph &g, int current_node);

bool canVisitClientDemand(Node *node, Vehicle &vehicle, Graph &g, int current_node);

bool canVisitClientBattery(Node *node, Vehicle &vehicle, Graph &g, int current_node);

bool solutionValid(Ant &ant, Graph &g);

Edge *closestClient(Graph &g, Node *current_node, Ant &ant);

#endif //DCC067_ACO_ACO_H
