#ifndef GRAPHS_NOMEQUIEROIRSENORGRAFO_NODE_H
#define GRAPHS_NOMEQUIEROIRSENORGRAFO_NODE_H
#include <iostream>
#include <fstream>
#include <cmath>
#include <set>
#include <map>
#include <unordered_map>

using namespace std;

template<typename G>
class Node {
public:
    typedef typename G::N N;
    typedef typename G::EdgeSeq EdgeSeq;
    typedef typename EdgeSeq::iterator EdgeIte;
    EdgeSeq edges;
    N data;
    string name;
    double x, y;

public:
    Node() : x(0), y(0) {}
    Node(N data) : data(data), x(0), y(0) {}
    Node(N data, double _x, double _y) : data(data), x(_x), y(_y) {}
    Node(Node* node){
        this->data = node->data;
        this->x = node->x;
        this->y = node->y;
    }
    int sizeEdges(){ return edges.size(); }
    bool operator<(const Node &other) const { return this->data < other.data; }

    bool operator==(const Node &other) const { return this->data == other.data; }

    EdgeIte firstEdge() {
        return edges.begin();
    }

    EdgeIte lastEdge() {
        return edges.end();
    }

    ~Node() {
        this->edges.clear();
    }
};

#endif //GRAPHS_NOMEQUIEROIRSENORGRAFO_NODE_H
