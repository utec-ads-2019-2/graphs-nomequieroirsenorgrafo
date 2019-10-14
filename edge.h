//
// Created by ASMX108 on 13/10/2019.
//

#ifndef GRAPHS_NOMEQUIEROIRSENORGRAFO_EDGE_H
#define GRAPHS_NOMEQUIEROIRSENORGRAFO_EDGE_H

template<typename G>
class Edge {
private:
    typedef typename G::E E;
    typedef typename G::node node;
    E data;
    node *nodes[2]; // Another option: node* from; node* to;
public:
    Edge() = default;

    explicit Edge(E data) : data{data}, nodes{nullptr} {}

    E getData() const { return this->data; }

    node *getFrom() const { return nodes[0]; }

    node *getTo() const { return nodes[1]; }

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
};


#endif //GRAPHS_NOMEQUIEROIRSENORGRAFO_EDGE_H
