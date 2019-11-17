// Created by felix on 10/10/19.
#ifndef GRAPHS_NOMEQUIEROIRSENORGRAFO_JSON_H
#define GRAPHS_NOMEQUIEROIRSENORGRAFO_JSON_H

#include "Graph.h"
#include "../vendor/nlohmann/json.hpp"

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

            newgraph->addVertex(airport.at("Id"), stof(x), stof(y));
        }

        //Add the edges
        for (auto it = airports.begin(); it != airports.end(); ++it) {
            auto airport = *it;

            auto tagFrom = airport.at("Id");

            auto destinations = airport.at("destinations");
            for (auto it2 = destinations.begin(); it2 != destinations.end(); ++it2){
                auto tagTo =  (*it2);
                //newgraph->addEdge( tagFrom,  tagTo);
                newgraph->addDirectedEdge( tagFrom,  tagTo);
            }
        }

        if (file_input.is_open())
            file_input.close();

        return *newgraph;
    }

    bool parseGraph(G graphParam) {
        json jsonGraph;
        //Nodes
        for(auto node = graphParam.firstNode() ; node != graphParam.lastNode() ; ++node) {
            auto vertex = (*node).second;

            //Edges
            json destinationsArray = json::array();
            auto nodeIt = (*node).second;
            for(auto edgeIt = nodeIt->firstEdge() ; edgeIt != nodeIt->lastEdge() ; edgeIt++) {
                destinationsArray.push_back( (*edgeIt)->nodes[1]->data);
            }

            json jsonAirport = {
                    {"Latitude", vertex->y},
                    {"Longitude", vertex->x},
                    {"Name",""},
                    {"Id", (*node).first},
                    {"destinations", destinationsArray}
            };
            jsonGraph.push_back(jsonAirport);
        }

        ofstream outputJson(this->fileName);
        outputJson << std::setw(4) << jsonGraph << endl;

        return true;
    }
};

#endif //GRAPHS_NOMEQUIEROIRSENORGRAFO_JSON_H
