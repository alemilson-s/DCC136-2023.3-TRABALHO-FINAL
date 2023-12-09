#include "Graph.h"
#include "Node.h"
#include "Edge.h"
#include <iostream>
#include <fstream>
#include <stack>
#include <queue>
#include <list>
#include <cstdlib>
#include <ctime>
#include <iomanip>
#include <random>
#include <chrono>


using namespace std::chrono;

using namespace std;

/**************************************************************************************************
 * Defining the Graph's methods
**************************************************************************************************/

Graph::Graph(int order, bool directed, bool weighted_edge, bool weighted_node) {

    this->order = order;
    this->directed = directed;
    this->weighted_edge = weighted_edge;
    this->weighted_node = weighted_node;
    this->first_node = this->last_node = nullptr;
    this->number_edges = 0;
    this->id_s = 0;
}


Graph::~Graph() {

    Node *next_node = this->first_node;

    while (next_node != nullptr) {

        next_node->removeAllEdges();
        Node *aux_node = next_node->getNextNode();
        delete next_node;
        next_node = aux_node;
    }
    this->order = 0;
}

int Graph::getOrder() {

    return this->order;
}

int Graph::getNumberEdges() {

    return this->number_edges;
}

bool Graph::getDirected() {

    return this->directed;
}

bool Graph::getWeightedEdge() {

    return this->weighted_edge;
}

bool Graph::getWeightedNode() {

    return this->weighted_node;
}

Node *Graph::getFirstNode() {

    return this->first_node;
}

Node *Graph::getLastNode() {

    return this->last_node;
}

Node *Graph::getNodeObjectId(unsigned int id) {
    Node *p = this->first_node;
    while (p != nullptr) {
        if (p->getObjectId() == id)
            return p;
        p = p->getNextNode();
    }
    return nullptr;
}

void Graph::insertNode(int id, bool update_order) {
    if (this->getFirstNode() == nullptr) {
        this->first_node = this->last_node = new Node(id);
        if (update_order)
            this->order++;
        this->first_node->setObjectId(this->id_s);
        id_s++;
    } else {
        if (!this->containsNode(id)) {
            Node *p = new Node(id);
            p->setObjectId(this->id_s);
            id_s++;
            this->last_node->setNextNode(p);
            this->last_node = p;
            this->last_node->setNextNode(nullptr);
            if (update_order)
                this->order++;
        }
    }
}

Node *Graph::allocateNode(int id, bool update_order) {
    /*******************************************************************************************************************
     * método similar ao insertNode(), se difere ao retornar o nó alocado. Assim tendo como objetivo retornar o nó
     * para que ele possa ser utilizado imediatamente e não haja necessidade de busca-lo no grafo.
     * ****************************************************************************************************************/
    if (this->getFirstNode() == nullptr) {
        this->first_node = this->last_node = new Node(id);
        if (update_order)
            this->order++;
        this->first_node->setObjectId(this->id_s);
        id_s++;
        return this->first_node;
    } else {
        if (!this->containsNode(id)) {
            Node *p = new Node(id);
            p->setObjectId(this->id_s);
            id_s++;
            this->last_node->setNextNode(p);
            this->last_node = p;
            this->last_node->setNextNode(nullptr);
            if (update_order)
                this->order++;
            return this->last_node;
        }
    }
    return nullptr;
}

void Graph::insertEdge(int id, int target_id, float weight, bool update_order) {
    if (id == target_id)
        return;
    Node *node;
    Node *target_node;
    if (!this->containsNode(id)) {
        node = this->allocateNode(id, update_order);
    } else {
        node = this->getNode(id);
    }
    if (!this->containsNode(target_id)) {
        target_node = this->allocateNode(target_id, update_order);
    } else {
        target_node = this->getNode(target_id);
    }

    if (this->getDirected()) {
        if (!node->containsEdge(id))
            node->insertEdge(target_id, weight);

        node->incrementOutDegree();
        target_node->incrementInDegree();
    } else {
        if (!node->containsEdge(target_id)) {
            node->insertEdge(target_id, weight);
            target_node->insertEdge(id, weight);
            node->incrementOutDegree();
            target_node->incrementOutDegree();
            node->incrementInDegree();
            target_node->incrementInDegree();
        }
    }
    this->number_edges++;
}

bool Graph::containsEdge(int id, int target_id) {
    Node *node = this->getNode(id);
    if (node == nullptr)
        return false;
    Edge *edge = node->getFirstEdge();

    while (edge != nullptr) {
        if (edge->getTargetId() == target_id) {
            return true;
        }
        edge = edge->getNextEdge();
    }

    return false;
}

void Graph::removeNode(int id) {
    /*******************************************************************************************************************
     * método utilizado para remover um nó do grafo.
     * parameters:
     *      id: id do nó que será removido do grafo
     * ****************************************************************************************************************/
    if (this->containsNode(id)) {
        Node *node = this->getNode(id);
        Edge *edge = node->getFirstEdge();
        while (edge != nullptr) {
            Edge *aux = edge;
            edge = edge->getNextEdge();
            node->removeEdge(this->directed, this->getNode(aux->getTargetId()));
        }

        Node *aux = this->getFirstNode();
        Node *previous = nullptr;

        while (aux->getId() != id) {
            previous = aux;
            aux = aux->getNextNode();
        }

        if (previous != nullptr) {
            previous->setNextNode(aux->getNextNode());
        } else {
            this->first_node = aux->getNextNode();
        }
        if (aux == this->last_node) {
            this->last_node = previous;
        }
        if (aux->getNextNode() == this->last_node) {
            previous->setNextNode(last_node);
        }
        this->order--;
    }

}

void Graph::removeEdge(int id, int target_id) {
    Node *node = this->getNode(id);
    node->removeEdge(this->directed, this->getNode(target_id));
    this->number_edges--;
}

bool Graph::containsNode(int id) {
    Node *node = this->getFirstNode();
    while (node != nullptr) {
        if (node->getId() == id)
            return true;
        node = node->getNextNode();
    }
    return false;
}

Node *Graph::getNode(int id) {
    Node *node = this->getFirstNode();

    while (node != nullptr) {
        if (node->getId() == id)
            return node;
        node = node->getNextNode();
    }
    return nullptr;
}

void Graph::generateDot(string name_graph) {
    /*******************************************************************************************************************
     * método utilizado para gerar o arquivo "*.dot".
     * parameters:
     *      name_graph: string utilizada como nome do arquivo que será gerado
     * ****************************************************************************************************************/
    string path = "../output_files/";
    path.append(name_graph).append("_graph.dot");
    fstream output_file;
    output_file.open(path, ios::out);
    if (!output_file.is_open()) {
        path = "./output_files/";
        path.append(name_graph).append("_graph.dot");
        output_file.open(path, ios::out);
    }

    if (output_file.is_open()) {
        if (this->directed) {
            output_file << "strict digraph " << name_graph << "{\n";
            for (Node *p = this->getFirstNode(); p != nullptr; p = p->getNextNode()) {
                output_file << p->getId() << "\n";// << " [pos=\"" << p->getX() << "," << p->getY() << "\"];\n";
            }
            for (Node *p = this->getFirstNode(); p != nullptr; p = p->getNextNode()) {
                for (Edge *t = p->getFirstEdge(); t != nullptr; t = t->getNextEdge()) {
                    if (this->weighted_edge) {
                        output_file << p->getId() << "->" << t->getTargetId() << "[weight=" << t->getWeight()
                                    << ", label=" << t->getWeight() << ", color=blue];\n";
                    } else {
                        output_file << p->getId() << "->" << t->getTargetId() << ";\n";
                    }

                }
            }
            output_file << "}";
        } else {
            output_file << "strict graph " << name_graph << "{\n";
            for (Node *p = this->getFirstNode(); p != nullptr; p = p->getNextNode()) {
                output_file << p->getId() << "\n";// << " [pos=\"" << p->getX() << "," << p->getY() << "\"];\n";
            }
            for (Node *p = this->getFirstNode(); p != nullptr; p = p->getNextNode()) {
                for (Edge *t = p->getFirstEdge(); t != nullptr; t = t->getNextEdge()) {
                    if (this->weighted_edge) {
                        output_file << p->getId() << "--" << t->getTargetId() << "[weight=" << t->getWeight()
                                    << ", label=" << t->getWeight() << ", color=blue];\n";
                    } else {
                        output_file << p->getId() << "--" << t->getTargetId() << ";\n";
                    }
                }
            }
            output_file << "}";
        }
        cout << "Arquivo " << path << " gerado!" << endl;
        name_graph.append("_graph.png");

        string command = "dot -Tpng ";
        command.append(path.append(" -o ../output_files/").append(name_graph));

        char *c = const_cast<char *>(command.c_str());

        if (!(c = reinterpret_cast<char *>((FILE *) popen(c, "r")))) {

            cout << "falha";

            exit(1);
        }
        output_file.close();

    } else {
        cout << "An error occurred while trying to open the file!" << endl << "generateDot()" << endl;
    }
}

void Graph::print() {
    Node *node = this->getFirstNode();
    while (node != nullptr) {
        cout << "\tNode id: " << node->getId() << endl;
        cout << "\tEdges:" << endl;
        Edge *edge = node->getFirstEdge();
        while (edge != nullptr) {
            cout << "\t\t(" << node->getId() << ", " << edge->getTargetId() << ") - " << edge->getWeight() << endl;
            edge = edge->getNextEdge();
        }
        node = node->getNextNode();
    }
    cout << endl;
}

int Graph::getVehicles() const {
    return vehicles;
}

void Graph::setVehicles(int vehicles) {
    Graph::vehicles = vehicles;
}

int Graph::getStations() const {
    return stations;
}

void Graph::setStations(int stations) {
    Graph::stations = stations;
}

float Graph::getCapacity() const {
    return capacity;
}

void Graph::setCapacity(float capacity) {
    Graph::capacity = capacity;
}

float Graph::getEnergyCapacity() const {
    return energy_capacity;
}

void Graph::setEnergyCapacity(float energyCapacity) {
    energy_capacity = energyCapacity;
}

float Graph::getEnergyConsumption() const {
    return energy_consumption;
}

void Graph::setEnergyConsumption(float energyConsumption) {
    energy_consumption = energyConsumption;
}





