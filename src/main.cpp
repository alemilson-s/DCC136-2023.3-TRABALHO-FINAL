#include <iostream>
#include "Graph.h"
#include "Node.h"
#include <fstream>
#include <string>
#include <sstream>
#include <vector>
#include <cmath>
#include "aco.h"

using namespace std;


double distance(Node &n, Node &m) {
    return sqrt((m.getX() - n.getX()) * (m.getX() - n.getX()) + (m.getY() - n.getY()) * (m.getY() - n.getY()));
}

float extractNumber(const std::string &texto) {
    size_t pos = texto.find(":");
    if (pos != std::string::npos && pos + 1 < texto.length()) {
        std::string numeroStr = texto.substr(pos + 1);
        return std::stod(numeroStr);
    }
    return -1;
}

Graph *leitura(ifstream &input_file) {
    Graph *g = new Graph(0, false, true, false);
    if (!input_file) {
        cout << "Arquivo não está aberto!" << endl;
        return nullptr;
    }
    string line;
    getline(input_file, line);
    for (int i = 0; i < 4; i++) {
        getline(input_file, line);
    }
    int vehicles = extractNumber(line);
    g->setVehicles(vehicles);
    getline(input_file, line);
    getline(input_file, line);
    int stations = extractNumber(line);
    g->setStations(stations);
    getline(input_file, line);
    int capacity = extractNumber(line);
    g->setCapacity(capacity);
    getline(input_file, line);
    int energy_capacity = extractNumber(line);
    g->setEnergyCapacity(energy_capacity);
    getline(input_file, line);
    float energy_consumption = extractNumber(line);
    g->setEnergyConsumption(energy_consumption);
    while (line != "NODE_COORD_SECTION ")
        getline(input_file, line);
    getline(input_file, line);
    int id;
    float x, y;
    while (line != "DEMAND_SECTION ") {
        istringstream iss(line);
        iss >> id;
        iss >> x;
        iss >> y;
        Node *node = g->allocateNode(id);
        node->setX(x);
        node->setY(y);
        getline(input_file, line);
    }
    getline(input_file, line);
    int demand;
    while (line != "STATIONS_COORD_SECTION ") {
        istringstream iss(line);
        iss >> id;
        iss >> demand;
        Node *node = g->getNode(id);
        node->setDemand(demand);
        getline(input_file, line);
    }
    getline(input_file, line);
    while (line != "DEPOT_SECTION") {
        istringstream iss(line);
        iss >> id;
        Node *node = g->getNode(id);
        node->setIsStation(true);
        node->setDemand(0);
        getline(input_file, line);
    }
    getline(input_file, line);
    Node *node = g->allocateNode(-1);
    istringstream iss_x(line);
    getline(input_file, line);
    istringstream iss_y(line);
    iss_x >> x;
    iss_y >> y;
    node->setX(x);
    node->setY(y);
    node->setIsStation(true);
    node->setDemand(0);
    node = g->getFirstNode();
    while (node != nullptr) {
        Node *prox = node->getNextNode();
        while (prox != nullptr) {
            double dist = distance(*node, *prox);
            node->insertEdge(prox->getId(), dist);
            prox->insertEdge(node->getId(), dist);
            prox = prox->getNextNode();
        }
        node = node->getNextNode();
    }
    return g;
}

int main(int argc, char **argv) {
    if (argc != 4 && argc != 2) {
        cout << "ERROR: Expecting: ./<program_name> <input_file> <directed>" << endl;
        cout << "Passar arquivo de entrada" << endl;
        return -1;
    }
    string file_name = argv[1];

    ifstream input_file;
    input_file.open(file_name, ios::in);

    Graph *g;
    g = leitura(input_file);
    if (input_file.is_open())
        input_file.close();
    int cycles = 20000;
    if (g->getOrder() >= 50)
        cycles = 70000;

    aco(*g, cycles, 0.7, 1, 9);
    delete g;
    return 0;

}
