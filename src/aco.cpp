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
                while (meetsDemand(ants[j].vehicles[l], g, ants[j])) {
                    Edge *next_client = selectNextClient(ants[j], ants[j].vehicles[l], g, alpha, beta,
                                                         0.9);
                    if (next_client == nullptr)
                        break;
                    Node *node = g.getNode(next_client->getTargetId());
                    ants[j].vehicles[l].path.push_back(next_client->getTargetId());
                    ants[j].visited[node->getObjectId()] = true;
                    ants[j].vehicles[l].battery -= g.getEnergyConsumption() * next_client->getWeight();
                    ants[j].vehicles[l].charge += node->getWeight();
                    ants[j].solution_value += next_client->getWeight();
                    double pheromone = (1 - evaporation) * next_client->getPheromone() +
                                       evaporation * (1.0 / (g.getOrder() * ants[j].solution_value));
                    next_client->setPheromone(pheromone);
                }
//                if(ants[j].vehicles[l].path.back() != ants[j].vehicles[l].path[0]){
//                    Edge *next_client = g.getNode(ants[j].vehicles[l].path.back())->getEdge(0);
//                    ants[j].vehicles[l].path.push_back(0);
//                    ants[j].vehicles[l].battery -= g.getEnergyConsumption() * next_client->getWeight();
//                    ants[j].solution_value += next_client->getWeight();
//                    double pheromone = (1 - evaporation) * next_client->getPheromone() +
//                                       evaporation * (1.0 / (g.getOrder() * ants[j].solution_value));
//                    next_client->setPheromone(pheromone);
//                }

            }
            if (ants[j].solution_value < best.solution_value && solutionValid(ants[j], g))
                best = ants[j];

            j++;
        }

        for (int i = 0; i < best.vehicles.size(); i++) {
            for (int k = 0; k < best.vehicles[i].path.size() - 1; k++) {
                Node *node = g.getNode(best.vehicles[i].path[k]);
                Edge *edge = node->getEdge(best.vehicles[i].path[k + 1]);
                double pheromone = (1 - evaporation) * edge->getPheromone() + evaporation * (1.0 / best.solution_value);
                edge->setPheromone(pheromone);
            }
        }
        t++;
    }
    cout << "Solução:" << endl;
    for (int i = 0; i < best.vehicles.size(); i++) {
        cout << "Veículo " << i << ": " << endl;
        for (int k = 0; k < best.vehicles[i].path.size(); k++) {
            cout << '\t' << '(' << best.vehicles[i].path[k] << ", " << best.vehicles[i].path[k + 1] << ')' << endl;
        }
    }
    cout << "Valor da solução: " << best.solution_value << endl;
}

void initializeAnts(Graph &g, vector<Ant> &ants, int n) {
    for (auto &ant: ants) {
        ant.visited.clear();
        ant.visited.resize(n, false);
        ant.vehicles.clear();
        ant.vehicles.resize(g.getVehicles());
        for (int j = 0; j < ant.vehicles.size(); j++) {
            ant.vehicles[j].battery = g.getEnergyCapacity();
            ant.vehicles[j].charge = 0;
            int number = closestClient(g, g.getNode(0), ant)->getTargetId();
            ant.vehicles[j].path.push_back(0);
            Node *node = g.getNodeObjectId(number);
            ant.vehicles[j].path.push_back(node->getId());
            ant.visited[node->getObjectId()] = true;
            ant.solution_value = node->getEdge(0)->getWeight();
            ant.vehicles[j].battery -= g.getNode(1)->getEdge(node->getId())->getWeight() * g.getEnergyConsumption();
            ant.vehicles[j].charge += node->getWeight();
        }
    }
}

void initializeParameters(vector<Ant> &ants, Graph &g) {
    Node *node = g.getFirstNode();
    while (node != nullptr) {
        Edge *edge = node->getFirstEdge();
        while (edge != nullptr) {
            float pheromone = 1.0 / edge->getWeight();
            edge->setPheromone(pheromone);
            edge = edge->getNextEdge();
        }
        node = node->getNextNode();
    }
    initializeAnts(g, ants, g.getOrder());
}

Edge *selectNextClient(Ant &ant, Vehicle &vehicle, Graph &g, float alpha, float beta, float q0) {
    Edge *client_list[g.getOrder() + g.getStations() * g.getOrder()];
    Edge *station_list[g.getOrder()];
    int n_clients = 0, n_stations = 0;

    int current_client = vehicle.path.back();

    Edge *edge = g.getNode(current_client)->getFirstEdge();

    double q = 0;
    while (edge != nullptr) {
        Node *node = g.getNode(edge->getTargetId());
        if (!ant.visited[node->getObjectId()] && canVisitClient(node, vehicle, g, current_client)) {
            q += pow(edge->getPheromone(), alpha) * pow(node->getWeight() / (edge->getWeight() + 1), beta);
            client_list[n_clients] = edge;
            n_clients++;
        }
        edge = edge->getNextEdge();
    }

    edge = g.getNode(current_client)->getFirstEdge();
    while (edge != nullptr) {
        Node *node = g.getNode(edge->getTargetId());
        if (canVisitStation(node, vehicle, g, current_client)) {
            station_list[n_stations] = edge;
            n_stations++;
        }
        edge = edge->getNextEdge();
    }

    vector<int> index_stations_clients;

    index_stations_clients.push_back(n_clients);
    for (int i = 0; i < n_stations; i++) {
        edge = g.getNode(station_list[i]->getTargetId())->getFirstEdge();
        while (edge != nullptr) {
            Node *node = g.getNode(edge->getTargetId());
            float battery = vehicle.battery;
            vehicle.battery = g.getEnergyCapacity();
            Node *node_current_client = g.getNode(current_client);
            if (!ant.visited[node->getObjectId()] && canVisitClient(node, vehicle, g, station_list[i]->getTargetId())) {
                q += pow(edge->getPheromone(), alpha) *
                     pow(node->getWeight() / (edge->getWeight() + 1 + station_list[i]->getWeight()), beta);
                client_list[n_clients] = edge;
                n_clients++;
            }
            vehicle.battery = battery;
            edge = edge->getNextEdge();
        }
        index_stations_clients.push_back(n_clients);
    }

    if (n_clients == 0 && current_client != 0) {
        return g.getNode(current_client)->getEdge(0);
    } else if (n_clients == 0 && current_client == 0)
        return nullptr;

    std::random_device rd_1;
    std::mt19937 generator(rd_1());
    std::uniform_real_distribution<double> distribution_1(0.0, 1.0);
    double random_number = distribution_1(generator);
    if (random_number < q0) {
        Node *node_better_heuristic = g.getNode(client_list[0]->getTargetId());
        Edge *edge_better_heuristic = client_list[0];
        int i = 0, index = 0;
        while (i < n_clients) {
            Node *n = g.getNode(client_list[i]->getTargetId());
            Edge *e = client_list[i];
            if (n->getWeight() / (e->getWeight() + 1) >
                node_better_heuristic->getWeight() / (edge_better_heuristic->getWeight() + 1)) {
                edge_better_heuristic = e;
                node_better_heuristic = n;
                index = i;
            }
            i++;
        }
        if (index >= index_stations_clients[0] && !index_stations_clients.empty()) {
            int index_station = 0;
            while (index_stations_clients[index_station] < index)
                index_station++;
            if (index_station == n_stations)
                index_station--;
            vehicle.path.push_back(station_list[index_station]->getTargetId());
            Node *node_current_client = g.getNode(current_client);
            if (vehicle.path.back() < 0)
                cout << "Aqui" << endl;
            vehicle.battery -=
                    node_current_client->getEdge(vehicle.path.back())->getWeight() * g.getEnergyConsumption();
            if (vehicle.battery < -1) {
                cout << "Battery restriction!" << endl;
            }
            vehicle.battery = g.getEnergyCapacity();
            ant.solution_value += node_current_client->getEdge(vehicle.path.back())->getWeight();
        }
        if (edge_better_heuristic == nullptr)
            cout << "NULL" << endl;
        return edge_better_heuristic;
    }

    vector<double> probabilities(n_clients, 0.0);

    for (int k = 0; k < n_clients; k++) {
        Node *node = g.getNode(client_list[k]->getTargetId());
        probabilities[k] = (pow(client_list[k]->getPheromone(), alpha) *
                            pow(node->getWeight() / (client_list[k]->getWeight() + 1), beta)) / q;
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
            if (i >= index_stations_clients[0]) {
                int index_station = 0;
                while (index_stations_clients[index_station] < i)
                    index_station++;
                if (index_station == n_stations)
                    index_station--;
                vehicle.path.push_back(station_list[index_station]->getTargetId());
                Node *node_current_client = g.getNode(current_client);
                vehicle.battery -=
                        node_current_client->getEdge(vehicle.path.back())->getWeight() * g.getEnergyConsumption();
                if (vehicle.battery < -1) {
                    cout << "Battery restriction!" << endl;
                }
                vehicle.battery = g.getEnergyCapacity();
                ant.solution_value += node_current_client->getEdge(vehicle.path.back())->getWeight();
            }
//            if(client_list[i])

            if (client_list[i] == nullptr)
                cout << "NULL" << endl;
            return client_list[i];
        }
    }
    return g.getNode(current_client)->getEdge(0);
}

bool canVisitClient(Node *node, Vehicle &vehicle, Graph &g, int current_node) {
    if (node->getId() == current_node || node->isStation())
        return false;

    if (!canVisitClientBattery(node, vehicle, g, current_node))
        return false;

    if (!canVisitClientDemand(node, vehicle, g, current_node))
        return false;
//    int current_charge = vehicle.charge;
//    int charge_cost = node->getWeight();
//    if (current_charge + charge_cost > g.getCapacity())
//        return false;
    float current_battery = vehicle.battery;
    float battery_cost = g.getEnergyConsumption() * node->getEdge(current_node)->getWeight();
    Edge *closestStation = closestStop(g, node);
    battery_cost += g.getEnergyConsumption() * node->getEdge(closestStation->getTargetId())->getWeight();
    if (current_battery - battery_cost < 0)
        return false;

    return true;
}

bool canVisitStation(Node *node, Vehicle &vehicle, Graph &g, int current_node) {
    if (node->getId() == current_node || !node->isStation())
        return false;

    float current_battery = vehicle.battery;
    float battery_cost = g.getEnergyConsumption() * node->getEdge(current_node)->getWeight();
    if (current_battery - battery_cost < 0)
        return false;

    if (node->getId() == 0)
        return true;

    battery_cost += g.getEnergyConsumption() * node->getEdge(0)->getWeight();
    if (g.getEnergyCapacity() - battery_cost < 0)
        return false;

    return true;
}

Edge *closestStop(Graph &g, Node *current_node) {
    Edge *edge = current_node->getFirstEdge();
    double closest = numeric_limits<float>::max();
    Edge *closestStop = nullptr;
    double distance;
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

bool meetsDemand(Vehicle &vehicle, Graph &g, Ant &ant) {
    Node *node = g.getNode(vehicle.path.back());
    Edge *edge = node->getFirstEdge();
    while (edge != nullptr) {
        Node *target = g.getNode(edge->getTargetId());
        if (!target->isStation() && canVisitClientDemand(target, vehicle, g, node->getId()) &&
            !ant.visited[target->getObjectId()])
            return true;
        edge = edge->getNextEdge();
    }
    return false;
}

bool canVisitClientDemand(Node *node, Vehicle &vehicle, Graph &g, int current_node) {
    if (node->getId() == current_node || node->isStation())
        return false;

    float current_charge = vehicle.charge;
    float charge_cost = node->getWeight();
    if (current_charge + charge_cost > g.getCapacity())
        return false;

    return true;
}

bool canVisitClientBattery(Node *node, Vehicle &vehicle, Graph &g, int current_node) {
    if (node->getId() == current_node || node->isStation())
        return false;

    float current_battery = vehicle.battery;
    float battery_cost = g.getEnergyConsumption() * node->getEdge(current_node)->getWeight();
    if (current_battery - battery_cost < 0)
        return false;
    return true;
}

bool solutionValid(Ant &ant, Graph &g) {
    vector<int> path;
    for (auto &vehicle: ant.vehicles) {
        for (int j: vehicle.path) {
            Node *node = g.getNode(j);
            if (node->isStation())
                continue;
            if (!ant.visited[node->getObjectId()])
                return false;
            else
                path.push_back(node->getId());
            for (int k: path) {
                if (k == node->getId()) {
                    return false;
                }
            }
        }
    }
    return true;
}

Edge *closestClient(Graph &g, Node *current_node, Ant &ant) {
    Edge *edge = current_node->getFirstEdge();
    double closest = numeric_limits<float>::max();
    Edge *closest_client = nullptr;
    double distance;
    // analisa todas as arestas do no que possivelmente será escolhido
    // retorna o hotel mais próximo a esse nó
    while (edge != nullptr) {
        Node *node = g.getNode(edge->getTargetId());
        // cálculo da distância
        distance = edge->getWeight();
        if (!node->isStation() && distance < closest && !ant.visited[node->getObjectId()]) {
            closest = distance;
            closest_client = edge;
        }
        edge = edge->getNextEdge();
    }
    return closest_client;
}