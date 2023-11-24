#include "Graph.h"
#include <queue>
#include <limits>
#include <valarray>
#include <iostream>
#include "aco.h"
#include <random>
#include <cmath>

using namespace std;

void aco(Graph &g, int cycles, float evaporation, float alpha, float beta) {
    cout << "Construindo solução ACO..." << endl;
    Ant best;
    int n_ants = g.getOrder() * 1 / 6;
    best.solution_value = numeric_limits<double>::max();
    vector<Ant> ants(n_ants, Ant());
    initializeParameters(ants, g, 10);
    cycles = cycles / (n_ants * n_ants);
    int t = 0;
    while (t < cycles) {
        initializeAnts(g, ants, g.getOrder());
        int j = 0;
        while (j < n_ants) {
            int k = 0;
            for (int l = 0; l < g.getVehicles(); l++) {
                Edge *next_city = selectNextCity(ants[j], ants[j].vehicles[l], g, alpha, beta);
                while (next_city->getTargetId() != g.getLastNode()->getId()) {
                    next_city = selectNextCity(ants[j], ants[j].vehicles[l], g, alpha, beta);
                    Node *node = g.getNode(next_city->getTargetId());
                    ants[j].vehicles[l].path.push_back(next_city->getTargetId());
                    if (!node->isStation()) {
                        ants[j].visited[node->getObjectId()] = true;
                        ants[j].solution_value += next_city->getWeight();
                    }
                }
                ants[j].vehicles[l].path.push_back(ants[j].vehicles[l].path[0]);
                Node *n = g.getNode(ants[j].vehicles[l].path[ants[j].vehicles[l].path.size() - 2]);
                Edge *e = n->getEdge(ants[j].vehicles[l].path.back());
                ants[j].solution_value += e->getWeight();
            }
            if (ants[j].solution_value < best.solution_value)
                best = ants[j];
//            cout << "Solução: " << best.solution_value << endl;
//            cout << "Solução: " << ants[j].solution_value << endl;
            j++;
        }
        for (j = 0; j < n_ants; j++) {
            for (int i = 0; i < ants[j].path.size() - 1; i++) {
                Node *node = g.getNode(ants[j].path[i]);
                Edge *edge = node->getEdge(ants[j].path[i + 1]);
                double pheromone = (1 - evaporation) * edge->getPheromone() +
                                   evaporation * (1.0 / (g.getOrder() * ants[j].solution_value));
                edge->setPheromone(pheromone);
                if (!g.getDirected()) {
                    node = g.getNode(ants[j].path[i + 1]);
                    edge = node->getEdge(ants[j].path[i]);
                    edge->setPheromone(pheromone);
                }
            }
        }
        for (int i = 0; i < best.path.size() - 1; i++) {
            Node *node = g.getNode(best.path[i]);
            Edge *edge = node->getEdge(best.path[i + 1]);
            double pheromone = (1 - evaporation) * edge->getPheromone() + evaporation * (1.0 / best.solution_value);
            edge->setPheromone(pheromone);
        }
//        cout << "próxima geração" << endl;
        t++;
    }
    cout << "Solução:" << endl;
    for (int i = 0; i < best.path.size() - 1; i++) {
        cout << '\t' << '(' << best.path[i] << ", " << best.path[i + 1] << ')' << endl;
    }
    cout << "Valor da solução: " << best.solution_value << endl;
}

void initializeAnts(Graph &g, vector<Ant> &ants, int n) {
    random_device rd;
    mt19937 gen(rd());
    uniform_int_distribution<int> dis(0, n - 1);
    for (int i = 0; i < ants.size(); i++) {
        ants[i].visited.clear();
        ants[i].visited.resize(n, false);
        for (int j = 0; j < ants[j].vehicles.size(); j++)
            ants[i].vehicles[j].path.clear();
        for (int j = 0; j < ants[j].vehicles.size(); j++) {
            ants[j].vehicles[j].battery = g.getEnergyCapacity();
            ants[j].vehicles[j].charge = 0;
            int aleatory_number = dis(rd);
            ants[i].vehicles[j].path.push_back(-1);
            Node *node = g.getNodeObjectId(aleatory_number);
            ants[i].vehicles[j].path.push_back(node->getId());
            ants[i].visited[node->getObjectId()] = true;
            ants[i].solution_value = g.getNode(-1)->getEdge(aleatory_number)->getWeight();
        }
    }
}

void initializeParameters(vector<Ant> &ants, Graph &g, float pheromone) {
    Node *node = g.getFirstNode();
    while (node != nullptr) {
        Edge *edge = node->getFirstEdge();
        while (edge != nullptr) {
            edge->setPheromone(pheromone);
            edge = edge->getNextEdge();
        }
        node = node->getNextNode();
    }
    initializeAnts(g, ants, g.getOrder());
}

Edge *selectNextCity(Ant &ant, Vehicle &vehicle, Graph &g, float alpha, float beta) {
    Edge *edges[g.getOrder()];
    int n_edges = 0;
    int current_city = vehicle.path.back();
    Edge *edge = g.getNode(current_city)->getFirstEdge();
    double q = 0;
    while (edge != nullptr) {
        Node *node = g.getNode(edge->getTargetId());
        if (!ant.visited[node->getObjectId()]) {
            q += pow(edge->getPheromone(), alpha) * pow(1.0 / (edge->getWeight() + 1), beta);
            edges[n_edges] = edge;
            n_edges++;
        }
        edge = edge->getNextEdge();
    }

    vector<double> probabilities(n_edges, 0.0);

    for (int k = 0; k < n_edges; k++) {
        probabilities[k] = (pow(edges[k]->getPheromone(), alpha) * pow(1.0 / (edges[k]->getWeight() + 1), beta)) / q;
    }

    double p[probabilities.size()];
    for (int i = 0; i < probabilities.size(); i++)
        p[i] = probabilities[i] * 10000;
    random_device rd;
    mt19937 gen(rd());
    uniform_real_distribution<double> distribution(0.0, 10000.0);
    double r = distribution(gen);
    double t = 0;
    for (int i = 0; i < probabilities.size(); i++) {
        t += p[i];
        if (t >= r) {
            return edges[i];
        }
    }
    return nullptr;
}

