//
// Created by ASMX108 on 13/10/2019.
//

#ifndef GRAPHS_NOMEQUIEROIRSENORGRAFO_NODE_H
#define GRAPHS_NOMEQUIEROIRSENORGRAFO_NODE_H

template<typename G>
class Node {
private:
    typedef typename G::N N;
    typedef typename G::EdgeSeq EdgeSeq;
    EdgeSeq edges;
    N data;
    double x, y; // Coordinates //RMP: maybe, it wont be necessary
public:
    Node() : x(0), y(0) {}

    Node(N data, double _x, double _y) : data{data}, x{_x}, y{_y} {}

    bool operator<(const Node &other) const { return this->data < other.data; }

    bool operator==(const Node &other) const { return this->data == other.data; }

    N getData() const { return this->data; }

    double getX() const { return this->x; }

    double getY() const { return this->y; }
};

#endif //GRAPHS_NOMEQUIEROIRSENORGRAFO_NODE_H
