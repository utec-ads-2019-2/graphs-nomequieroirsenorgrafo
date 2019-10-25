#include "Graph.h"
#include "Json.h"

using namespace std;

int main() {
    graph testBfs;

    testBfs.addVertex("v", 0, 0);
    testBfs.addVertex("r", 0, 1);
    testBfs.addVertex("s", 1, 1);
    testBfs.addVertex("w", 1, 0);
    testBfs.addVertex("x", 2, 0);
    testBfs.addVertex("t", 2, 1);
    testBfs.addVertex("y", 3, 0);
    testBfs.addVertex("u", 3, 1);

    testBfs.addEdge("v", "r");testBfs.addEdge("r", "v");
    testBfs.addEdge("r", "s");testBfs.addEdge("s", "r");
    testBfs.addEdge("s", "w");testBfs.addEdge("w", "s");
    testBfs.addEdge("w", "x");testBfs.addEdge("x", "w");
    testBfs.addEdge("w", "t");testBfs.addEdge("t", "w");
    testBfs.addEdge("x", "t");testBfs.addEdge("t", "x");
    testBfs.addEdge("x", "u");testBfs.addEdge("u", "x");
    testBfs.addEdge("x", "y");testBfs.addEdge("y", "x");
    testBfs.addEdge("t", "u");testBfs.addEdge("u", "t");
    testBfs.addEdge("u", "y");testBfs.addEdge("y", "u");

    testBfs.printGraph();

    testBfs.bfs("s");
    auto connected = ( testBfs.isConnected() == true )?"Si":"No";
    cout<< "Is connected -> "<<connected<< endl;

    graph testBfs2;

    testBfs2.addVertex("9", 0, 0);
    testBfs2.addVertex("26", 0, 0);
    testBfs2.addVertex("11", 0, 0);
    testBfs2.addVertex("18", 0, 0);
    testBfs2.addVertex("19", 0, 0);
    testBfs2.addVertex("13", 0, 0);
    testBfs2.addVertex("5", 0, 0);
    testBfs2.addVertex("17", 0, 0);
    testBfs2.addVertex("23", 0, 0);
    testBfs2.addVertex("24", 0, 0);

    testBfs2.addEdge("9", "26");testBfs2.addEdge("26", "9");
    testBfs2.addEdge("9", "19");testBfs2.addEdge("19", "9");
    testBfs2.addEdge("9", "18");testBfs2.addEdge("18", "9");
    testBfs2.addEdge("26", "11");testBfs2.addEdge("11", "26");
    testBfs2.addEdge("9", "19");testBfs2.addEdge("19", "9");
    testBfs2.addEdge("13", "19");testBfs2.addEdge("19", "13");
    testBfs2.addEdge("13", "5");testBfs2.addEdge("5", "13");
    testBfs2.addEdge("23", "24");testBfs2.addEdge("24", "23");

    auto connected2 = ( testBfs2.isConnected() == true )?"Si":"No";
    cout<< "Graph 2 Is connected? -> "<<connected2<< endl;

    /*graph test;
    test.addNode("a",0.1, 0.2);
    test.addNode("b",0.2, 0.3);
    test.addNode("c",0.4, 0.5);
    test.addVertex("d",0.7, 0.6);

    test.addEdge("a", "b");
    test.addEdge("b", "c");
    test.addEdge("c", "d");
    test.addEdge("d", "a");

    test.printGraph(); cout << endl;

    auto primGraph = test.prim("a");
    primGraph.printGraph();*/

//   auto read = new Json<graph>("airportsTest.json");
/*    auto read = new Json<graph>("airports.json");
    read->parseJson();*/

    return 0;
}


