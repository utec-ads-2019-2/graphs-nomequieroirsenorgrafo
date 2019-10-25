#ifndef GRAPHS_NOMEQUIEROIRSENORGRAFO_EDGE_H
#define GRAPHS_NOMEQUIEROIRSENORGRAFO_EDGE_H

#include <bits/stdc++.h>
using namespace std;

template<typename G>
class Edge {
private:
    typedef typename G::E E;
    typedef typename G::node node;
public:
    E data;
    node *nodes[2]; // Another option: node* from; node* to;

    Edge() = default;

    Edge(E data) : data(data) {

    }

    E getData() const { return this->data; }

    std::pair<node, node> getNodes() const { return {nodes[0], nodes[1]}; }

    bool operator<(const Edge &other) const {
        if (this->data != other.data)
            return this->data < other.data;

        if (nodes[0] != other.from)
            return nodes[0] < other.from;

        return nodes[1] < other.nodes[1];
    }

    bool operator==(const Edge &other) const {
        return nodes[0] == other.nodes[0] and nodes[1] == other.nodes[1];
    }

    ~Edge() {
        delete [] nodes;
    }
};
#endif //GRAPHS_NOMEQUIEROIRSENORGRAFO_EDGE_H
