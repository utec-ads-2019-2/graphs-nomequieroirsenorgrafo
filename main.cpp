#include "Graph.h"
#include "Json.h"

using namespace std;

void graphCormen(graph);
void graphTestBfs(graph);

int main() {
    /*Cormen graph*/
    graph cormen;
    graphCormen(cormen);

    /*TestBfs graph*/
    /*graph testBfs;
    graphTestBfs(testBfs);

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
    testBfs2.privateAddEdge("13", "19");testBfs2.addEdge("19", "13");
    testBfs2.addEdge("13", "5");testBfs2.addEdge("5", "13");
    testBfs2.addEdge("23", "24");testBfs2.addEdge("24", "23");

    auto connected2 = ( testBfs2.isConnected() == true )?"Si":"No";
    cout<< "Graph 2 Is connected? -> "<<connected2<< endl;*/

    /*graph cormen;
    cormen.addNode("a",0.1, 0.2);
    cormen.addNode("b",0.2, 0.3);
    cormen.addNode("c",0.4, 0.5);
    cormen.addVertex("d",0.7, 0.6);

    cormen.addEdge("a", "b");
    cormen.privateAddEdge("b", "c");
    cormen.addEdge("c", "d");
    cormen.addEdge("d", "a");

    cormen.printGraph(); cout << endl;

    auto primGraph = cormen.prim("a");
    primGraph.printGraph();*/

//   auto read = new Json<graph>("airportsTest.json");
/*    auto read = new Json<graph>("airports.json");
    read->parseJson();*/

    return 0;
}

void graphCormen(graph cormen)
{
    cormen.addVertex("a"); cormen.addVertex("b"); cormen.addVertex("c");
    cormen.addVertex("d"); cormen.addVertex("e"); cormen.addVertex("f");
    cormen.addVertex("g"); cormen.addVertex("h"); cormen.addVertex("i");

    cormen.addEdge("a", "b", 4); cormen.addEdge("b", "a", 4);
    cormen.addEdge("a", "h", 8); cormen.addEdge("h", "a", 8);

    cormen.addEdge("b", "c", 8); cormen.addEdge("c", "b", 8);
    cormen.addEdge("b", "h", 11); cormen.addEdge("h", "b", 11);

    cormen.addEdge("c", "d", 7); cormen.addEdge("d", "c", 7);
    cormen.addEdge("c", "i", 2); cormen.addEdge("i", "c", 2);
    cormen.addEdge("c", "f", 4); cormen.addEdge("f", "c", 4);

    cormen.addEdge("d", "e", 9); cormen.addEdge("e", "d", 9);
    cormen.addEdge("d", "f", 14); cormen.addEdge("f", "d", 14);

    cormen.addEdge("e", "f", 10); cormen.addEdge("f", "e", 10);

    cormen.addEdge("f", "g", 2); cormen.addEdge("g", "f", 2);

    cormen.addEdge("g", "h", 1); cormen.addEdge("h", "g", 1);
    cormen.addEdge("g", "i", 6); cormen.addEdge("i", "g", 6);

    cormen.addEdge("h", "i", 7); cormen.addEdge("i", "h", 7);

    cout << cormen.findEdge("g", "h") << endl;
//    cout << cormen.deleteEdge("g", "h") << endl;
    cormen.printGraph(); cout << endl;

    auto primCormen = cormen.prim("a");
    primCormen.printGraph();
}

void graphTestBfs(graph testBfs)
{
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

}
