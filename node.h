#ifndef GRAPHS_NOMEQUIEROIRSENORGRAFO_NODE_H
#define GRAPHS_NOMEQUIEROIRSENORGRAFO_NODE_H
#include <iostream>
#include <fstream>
#include <cmath>
#include <set>
#include <map>

using namespace std;

template<typename G>
class Node {
public:
    typedef typename G::N N;
    typedef typename G::EdgeSeq EdgeSeq;
private:
    EdgeSeq edges;
    N data;
    double x, y;
public:
    Node() : x(0), y(0) {}

    Node(N data, double _x, double _y) : data(data), x(_x), y(_y) {}

    bool operator<(const Node &other) const { return this->data < other.data; }

    bool operator==(const Node &other) const { return this->data == other.data; }

    N getData() const { return this->data; }

    double getX() const { return this->x; }

    double getY() const { return this->y; }

};

#endif //GRAPHS_NOMEQUIEROIRSENORGRAFO_NODE_H
