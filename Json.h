// Created by felix on 10/10/19.
#ifndef GRAPHS_NOMEQUIEROIRSENORGRAFO_JSON_H
#define GRAPHS_NOMEQUIEROIRSENORGRAFO_JSON_H

#include "Graph.h"
#include "nlohmann/json.hpp"

using json = nlohmann::json;
template <typename G>
class Json{
private:
    string fileName;
public:
    typedef typename G::N N;
    typedef typename G::E E;

    Json(string fileName) {
        this->fileName = fileName;
    }
//  Process a JSON and return a Graph
    graph &parseJson() {
        ifstream file_input(this->fileName);
        json aeroportos;

        if (!file_input)
            throw runtime_error("The file doesn't exist");

        file_input >> aeroportos;

        auto jsonSize = aeroportos.size();
        if (jsonSize <= 10)
            for (json::iterator it = aeroportos.begin(); it != aeroportos.end(); ++it) {
                cout << *it << endl;
            }
        else
            cout << jsonSize << " aeropuertos" << endl;
    }
//  Process a Graph and return a JSON
    void parseGraph() {
//  RMP: return void for now.
    }
};

#endif //GRAPHS_NOMEQUIEROIRSENORGRAFO_JSON_H
