#ifndef SRC_NODE_H
#define SRC_NODE_H

#include "Edge.h"

using namespace std;

class Node {

private:
    Edge *first_edge;
    Edge *last_edge;
    unsigned int object_id;
    int id;
    unsigned int in_degree;
    unsigned int out_degree;
    float weight;
    Node *next_node;
    double x;
    double y;
    bool is_station;

public:

    Node(int id);

    Node(int id, double xl, double yl);

    Node();

    ~Node();

    double getX() const;

    void setX(double x);

    double getY() const;

    void setY(double y);

    Edge *getFirstEdge();

    Edge *getLastEdge();

    int getId();

    int getInDegree();

    int getOutDegree();

    float getWeight();

    Node *getNextNode();

    Edge *getEdge(int target_id);

    unsigned int getObjectId();

    void setNextNode(Node *node);

    void setWeight(float weight);

    void setObjectId(unsigned int object_id);

    bool searchEdge(int target_id);

    void insertEdge(int target_id, double weight, bool update_order = false);

    void removeAllEdges();

    int removeEdge(bool directed, Node *target_node);

    void incrementOutDegree();

    void decrementOutDegree();

    void incrementInDegree();

    void decrementInDegree();

    Edge *hasEdgeBetween(int target_id);

    bool containsEdge(int edge_id);

    void auxRemoveEdge(Node *node, int id_edge);

    bool isStation() const;

    void setIsStation(bool isStation);
};


#endif