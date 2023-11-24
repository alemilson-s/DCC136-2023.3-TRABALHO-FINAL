#ifndef SRC_EDGE_H
#define SRC_EDGE_H


class Edge {
private:
    int target_id;
    Edge *next_edge;
    double weight;
    double pheromone;

public:
    Edge(int target_id);

    ~Edge();

    int getTargetId();

    Edge *getNextEdge();

    double getWeight();

    double getPheromone() const;

    void setNextEdge(Edge *edge);

    void setWeight(double weight);

    void setPheromone(double pheromone);

};


#endif
