// Created by felix on 10/10/19.
#ifndef GRAPHS_NOMEQUIEROIRSENORGRAFO_JSON_H
#define GRAPHS_NOMEQUIEROIRSENORGRAFO_JSON_H

#include "Graph.h"
#include "nlohmann/json.hpp"

using json = nlohmann::json;

template<typename G>
class Json {
private:
    string fileName;
public:
    typedef typename G::N N;
    typedef typename G::E E;

    Json(string fileName) {
        this->fileName = fileName;
    }

//  Process a JSON and return a Graph
    graph parseJson() {
        ifstream file_input(this->fileName);
        json airports;

        if (!file_input)
            throw runtime_error("The JSON file doesn't exist");
        file_input >> airports;

        auto jsonSize = airports.size();

        auto newgraph = new graph();
        //Add the nodes
        for (json::iterator it = airports.begin(); it != airports.end(); ++it) {
            auto airport = *it;

            auto x = airport.at("Latitude").get<string>();
            auto y = airport.at("Longitude").get<string>();

            newgraph->addNode(airport.at("Id"), stof( x ), stof( y ) );
        }

        //Add the edges
        for (json::iterator it = airports.begin(); it != airports.end(); ++it) {
            auto airport = *it;

            auto tagFrom = airport.at("Id");

            auto destinations = airport.at("destinations");
            for (json::iterator it2 = destinations.begin(); it2 != destinations.end(); ++it2){
                auto tagTo =  (*it2);
                newgraph->addEdge( tagFrom,  tagTo);
            }
        }

        if (file_input.is_open())
            file_input.close();

        cout << "Nodes -> " << newgraph->getNumberOfNodes() << endl;
        cout << "Edges -> " << newgraph->getNumberOfEdges() << endl;

        newgraph->deleteNode("4027");
        cout << "Del aeropuerto/node 4027" << endl;
        //cout << "Delete egde 4027 -> 3536 " << newgraph->deleteEdge("4027", "3536") << endl;
        //cout << "Delete egde 4024 -> 1767 " << newgraph->deleteEdge("4024", "1767") << endl;

        cout << "Nodes -> " << newgraph->getNumberOfNodes() << endl;
        cout << "Edges -> " << newgraph->getNumberOfEdges() << endl;
        printf("Densidad: %4.7f", newgraph->getDensity() );

        newgraph->recorridoMinimapita("1767");
        return *newgraph;
    }

/*    graph& getGraph() {

    }*/

    void parseGraph() {
//  RMP: return void for now.
    }
};

#endif //GRAPHS_NOMEQUIEROIRSENORGRAFO_JSON_H
