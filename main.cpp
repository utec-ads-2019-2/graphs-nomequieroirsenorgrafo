#include "Graph.h"
#include "Json.h"

using namespace std;

int main() {
    /*graph test;
    test.addNode("a",0.1, 0.2);
    test.addNode("b",0.2, 0.3);
    test.addNode("c",0.4, 0.5);
    test.addNode("d",0.7, 0.6);

    test.addEdge("a", "b");
    test.addEdge("b", "c");
    test.addEdge("c", "d");
    test.addEdge("d", "a");

    test.printGraph(); cout << endl;

    auto primGraph = test.prim("a");
    primGraph.printGraph();*/

//   auto read = new Json<graph>("airportsTest.json");
    auto read = new Json<graph>("airports.json");
    read->parseJson();

    return 0;
}


