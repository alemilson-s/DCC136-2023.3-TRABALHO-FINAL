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
    initializeParameters(ants, g);
    cycles = cycles / (n_ants * n_ants);
    int t = 0;
    while (t < cycles) {
        initializeAnts(g, ants, g.getOrder());
        int j = 0;
        while (j < n_ants) {
            for (int l = 0; l < g.getVehicles(); l++) {
                Edge *next_client;
                while (ants[j].vehicles[l].path.back() != ants[j].vehicles[l].path[0]) {
                    next_client = selectNextClient(ants[j], ants[j].vehicles[l], g, alpha, beta, 0.9);
                    Node *node = g.getNode(next_client->getTargetId());
                    ants[j].vehicles[l].path.push_back(next_client->getTargetId());
                    if (!node->isStation()) {
                        ants[j].visited[node->getObjectId()] = true;
                    }
                    ants[j].solution_value += next_client->getWeight();
                    double pheromone = (1 - evaporation) * next_client->getPheromone() +
                                       evaporation * (1.0 / (g.getOrder() * ants[j].solution_value));
                    next_client->setPheromone(pheromone);
                }
//                ants[j].vehicles[l].path.push_back(ants[j].vehicles[l].path[0]);
//                Node *n = g.getNode(ants[j].vehicles[l].path[ants[j].vehicles[l].path.size() - 2]);
//                Edge *e = n->getEdge(ants[j].vehicles[l].path.back());
//                ants[j].solution_value += e->getWeight();
            }
            if (ants[j].solution_value < best.solution_value)
                best = ants[j];
//            cout << "Solução: " << best.solution_value << endl;
//            cout << "Solução: " << ants[j].solution_value << endl;
            j++;
        }
        for (int i = 0; i < best.vehicles.size(); i++) {
            for (int k = 0; k < best.vehicles[i].path.size() - 1; k++) {
                Node *node = g.getNode(best.vehicles[i].path[k]);
                Edge *edge = node->getEdge(best.vehicles[i].path[i + 1]);
                double pheromone = (1 - evaporation) * edge->getPheromone() + evaporation * (1.0 / best.solution_value);
                edge->setPheromone(pheromone);
            }
        }
//        cout << "próxima geração" << endl;
        t++;
    }
    cout << "Solução:" << endl;
    for (int i = 0; i < best.vehicles.size(); i++) {
        cout << "Veículo" << i << ": " << endl;
        for (int k = 0; k < best.vehicles[i].path.size(); k++) {
            cout << '\t' << '(' << best.vehicles[i].path[i] << ", " << best.vehicles[i].path[i + 1] << ')' << endl;
        }
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
        ants[i].vehicles.resize(g.getVehicles());
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

void initializeParameters(vector<Ant> &ants, Graph &g) {
    Node *node = g.getFirstNode();
    while (node != nullptr) {
        Edge *edge = node->getFirstEdge();
        while (edge != nullptr) {
            float pheromone = 1.0f / edge->getWeight();
            edge->setPheromone(pheromone);
            edge = edge->getNextEdge();
        }
        node = node->getNextNode();
    }
    initializeAnts(g, ants, g.getOrder());
}

Edge *selectNextClient(Ant &ant, Vehicle &vehicle, Graph &g, float alpha, float beta, float q0) {
    Edge *edges[g.getOrder()];
    int n_edges = 0;
    int current_client = vehicle.path.back();
    Edge *edge = g.getNode(current_client)->getFirstEdge();
    Edge *edge_better_heuristic = edge;
    Node *node_better_heuristic = g.getNode(edge_better_heuristic->getTargetId());
    double q = 0;
    while (edge != nullptr) {
        Node *node = g.getNode(edge->getTargetId());
        if (!ant.visited[node->getObjectId()] && canVisit(node, vehicle, g, current_client)) {
            q += pow(edge->getPheromone(), alpha) * pow(node->getWeight() / (edge->getWeight() + 1), beta);
            edges[n_edges] = edge;
            n_edges++;
            if (node->getWeight() / (edge->getWeight() + 1) >
                node_better_heuristic->getWeight() / edge_better_heuristic->getWeight()) {
                node_better_heuristic = node;
                edge_better_heuristic = edge;
            }
        }
        edge = edge->getNextEdge();
    }

    std::random_device rd_1;
    std::mt19937 generator(rd_1());
    std::uniform_real_distribution<double> distribution_1(0.0, 1.0);
    double random_number = distribution_1(generator);
    if (random_number < q0) {
        return edge_better_heuristic;
    }

    vector<double> probabilities(n_edges, 0.0);

    for (int k = 0; k < n_edges; k++) {
        Node *node = g.getNode(edges[k]->getTargetId());
        probabilities[k] = (pow(edges[k]->getPheromone(), alpha) *
                            pow(node->getWeight() / (edges[k]->getWeight() + 1), beta)) / q;
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
    return g.getNode(current_client)->getEdge(1);
}

bool canVisit(Node *node, Vehicle &vehicle, Graph &g, int current_node) {
    int current_battery = vehicle.battery;
    int battery_cost = g.getEnergyConsumption() * node->getEdge(current_node)->getWeight();
    if (battery_cost > current_battery)
        return false;

    int current_charge = vehicle.charge;
    int charge_cost = node->getWeight();
    if (charge_cost > current_charge)
        return false;

    Edge *closestStation = closestStop(g, node);
    battery_cost += g.getEnergyConsumption() * node->getEdge(closestStation->getTargetId())->getWeight();
    if (battery_cost > current_battery)
        return false;

    return true;
}

Edge *closestStop(Graph &g, Node *current_node) {
    Edge *edge = current_node->getFirstEdge();
    float closest = numeric_limits<float>::max();
    Edge *closestStop = nullptr;
    float distance;
    // analisa todas as arestas do no que possivelmente será escolhido
    // retorna o hotel mais próximo a esse nó
    while (edge != nullptr) {
        Node *node = g.getNode(edge->getTargetId());
        // cálculo da distância
        distance = edge->getWeight();
        if (node->isStation() && distance < closest) {
            closest = distance;
            closestStop = edge;
        }
        edge = edge->getNextEdge();
    }
    return closestStop;
}

