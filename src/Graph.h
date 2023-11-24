#ifndef SRC_GRAPH_H
#define SRC_GRAPH_H

#include "Node.h"
#include <fstream>
#include <stack>
#include <list>
#include <vector>

class Graph {

private:
    int order;
    int number_edges;
    bool directed;
    bool weighted_edge;
    bool weighted_node;
    Node *first_node;
    Node *last_node;
    int id_s;
    int vehicles;
    int stations;
    int capacity;
    int energy_capacity;
    float energy_consumption;

public:
    int getVehicles() const;

    void setVehicles(int vehicles);

    int getStations() const;

    void setStations(int stations);

    int getCapacity() const;

    void setCapacity(int capacity);

    int getEnergyCapacity() const;

    void setEnergyCapacity(int energyCapacity);

    float getEnergyConsumption() const;

    void setEnergyConsumption(float energyConsumption);

    Graph(int order, bool directed, bool weighted_edge, bool weighted_node);

    ~Graph();

    int getOrder();

    int getNumberEdges();

    bool getDirected();

    bool getWeightedEdge();

    bool getWeightedNode();

    Node *getFirstNode();

    Node *getLastNode();

    Node *getNodeObjectId(unsigned int id);

    void insertNode(int id, bool update_order = true);

    Node *allocateNode(int id, bool update_order = true);

    void insertEdge(int id, int target_id, float weight = 0, bool update_order = false);

    void removeNode(int id);

    void removeEdge(int id, int target_id);

    bool containsNode(int id);

    Node *getNode(int id);

    void generateDot(string name_graph);

    void print();

private:

    bool containsEdge(int id, int target_id);

};

#endif