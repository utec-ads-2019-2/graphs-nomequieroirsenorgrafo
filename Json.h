// Created by felix on 10/10/19.
#ifndef GRAPHS_NOMEQUIEROIRSENORGRAFO_JSON_H
#define GRAPHS_NOMEQUIEROIRSENORGRAFO_JSON_H

#include "Graph.h"
template <typename G>
class Json{
    typedef typename G::N N;
    typedef typename G::E E;

public:
    explicit Json(char* file) {

    }
//  Process a JSON and return a Graph
    graph& parseJson() {

    }
//  Process a Graph and return a JSON
    void parseGraph() {
//  RMP: return void for now.
    }
};

#endif //GRAPHS_NOMEQUIEROIRSENORGRAFO_JSON_H
