#include "Node.h"
#include "Edge.h"
#include <iostream>

using namespace std;

/**************************************************************************************************
 * Defining the Node's methods
**************************************************************************************************/

Node::Node(int id) {

    this->id = id;
    this->in_degree = 0;
    this->out_degree = 0;
    this->weight = 0;
    this->first_edge = nullptr;
    this->last_edge = nullptr;
    this->next_node = nullptr;
    this->x = 0;
    this->y = 0;
    this->is_station = false;
}

Node::Node(int id, double xl, double yl) {
    this->id = id;
    this->in_degree = 0;
    this->out_degree = 0;
    this->weight = 0;
    this->first_edge = nullptr;
    this->last_edge = nullptr;
    this->next_node = nullptr;
    this->x = xl;
    this->y = yl;
}

Node::~Node() {

    Edge *next_edge = this->first_edge;

    while (next_edge != nullptr) {

        Edge *aux_edge = next_edge->getNextEdge();
        delete next_edge;
        next_edge = aux_edge;

    }

}

double Node::getX() const {
    return this->x;
}

void Node::setX(double xl) {
    this->x = xl;
}

double Node::getY() const {
    return y;
}

void Node::setY(double yl) {
    this->y = yl;
}


Edge *Node::getFirstEdge() {

    return this->first_edge;

}

Edge *Node::getLastEdge() {

    return this->last_edge;

}

int Node::getId() {

    return this->id;

}

int Node::getInDegree() {

    return this->in_degree;

}

int Node::getOutDegree() {

    return this->out_degree;

}

float Node::getWeight() {

    return this->weight;

}

Node *Node::getNextNode() {

    return this->next_node;

}

Edge *Node::getEdge(int target_id) {
    Edge *edge = this->getFirstEdge();
    while (edge != nullptr) {
        if (edge->getTargetId() == target_id) {
            return edge;
        }
        edge = edge->getNextEdge();
    }
    return nullptr;
}

unsigned int Node::getObjectId() {
    return this->object_id;
}

void Node::setNextNode(Node *next_node) {

    this->next_node = next_node;

}

void Node::setObjectId(unsigned int object_id) {
    this->object_id = object_id;
}

void Node::setWeight(float weight) {

    this->weight = weight;

}

void Node::insertEdge(int target_id, double weight, bool update_order) {
    if (this->first_edge != nullptr) {
        Edge *edge = new Edge(target_id);
        edge->setWeight(weight);
        this->last_edge->setNextEdge(edge);
        this->last_edge = edge;

    } else {
        this->first_edge = new Edge(target_id);
        this->first_edge->setWeight(weight);
        this->last_edge = this->first_edge;

    }
}

void Node::removeAllEdges() {
    /*******************************************************************************************************************
     * método utilizado para remover todas arestas de um nó, é utilizado pelo destrutor.
     * ****************************************************************************************************************/
    if (this->first_edge != nullptr) {

        Edge *next;
        Edge *aux = this->first_edge;
        while (aux != nullptr) {

            next = aux->getNextEdge();
            delete aux;
            aux = next;
        }

    }

    this->first_edge = this->last_edge = nullptr;

}

void Node::auxRemoveEdge(Node *node, int id_edge) {
    /*******************************************************************************************************************
     * O método remove uma aresta adjacente a um nó
     * parameters:
     *      *node: ponteiro para o nó que terá aresta removida
     *      id_edge: id da aresta que removida
     * ****************************************************************************************************************/
    Edge *aux = node->first_edge;
    Edge *previous = nullptr;
    while (aux->getTargetId() != id_edge) {

        previous = aux;
        aux = aux->getNextEdge();

    }
    if (previous != nullptr)
        previous->setNextEdge(aux->getNextEdge());
    else
        node->first_edge = aux->getNextEdge();

    if (aux == node->last_edge)
        node->last_edge = previous;

    if (aux->getNextEdge() == node->last_edge)
        node->last_edge = aux->getNextEdge();
    aux->setNextEdge(nullptr);
    delete aux;
}

int Node::removeEdge(bool directed, Node *target_node) {
    /*******************************************************************************************************************
     * o método remove aresta adjacente a nós ou arcos, é utilizado para controlar a remoção de arestas em nós adjacentes
     * no caso de o grafo não ser direcionado, onde a aresta terá remoção na lista de adjacência de ambos nós.
     * parameters:
     *      directed: indica se o nó está em grafo direcionado
     *      *target_node: ponteiro para o nó adjacente ao objeto
     ******************************************************************************************************************/
    if (this->containsEdge(target_node->getId())) {
        auxRemoveEdge(this, target_node->getId());
        if (directed) {
            this->decrementOutDegree();
            target_node->decrementInDegree();
        } else {
            auxRemoveEdge(target_node, this->getId());
            this->decrementInDegree();
            target_node->decrementInDegree();

            this->decrementOutDegree();
            target_node->decrementOutDegree();

        }

        return 1;

    }

    return 0;

}

bool Node::searchEdge(int target_id) {
    if (this->first_edge != nullptr) {
        for (Edge *aux = this->first_edge; aux != nullptr; aux = aux->getNextEdge())
            if (aux->getTargetId() == target_id)
                return true;

    }

    return false;

}

void Node::incrementInDegree() {

    this->in_degree++;

}

void Node::incrementOutDegree() {

    this->out_degree++;

}

void Node::decrementInDegree() {

    this->in_degree--;

}

void Node::decrementOutDegree() {

    this->out_degree--;

}

Edge *Node::hasEdgeBetween(int target_id) {

    for (Edge *edge = this->first_edge; edge != nullptr; edge = edge->getNextEdge()) {
        if (edge->getTargetId() == target_id)
            return edge;
    }
    return nullptr;
}

bool Node::containsEdge(int edge_id) {
    /*******************************************************************************************************************
     * método utilizado para verificar se uma determinada aresta está contida no conjunto de arestas do nó.
     * parameters:
     *      edge_id: id da aresta que será analisada pelo método
     * return:
     *      retorna verdadeiro se a aresta estiver contida no conjunto de arestas do nó, falso caso contrário
    *******************************************************************************************************************/
    Edge *edge = this->first_edge;
    while (edge != nullptr) {
        if (edge->getTargetId() == edge_id) {
            return true;
        }
        edge = edge->getNextEdge();
    }
    return false;
}

int Node::getDemand() const {
    return demand;
}

void Node::setDemand(int demand) {
    Node::demand = demand;
}

bool Node::isStation() const {
    return is_station;
}

void Node::setIsStation(bool isStation) {
    is_station = isStation;
}
