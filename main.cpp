#include "Graph.h"
#include "Json.h"
#include "test/tester.h"

using namespace std;

int main() {

//   auto read = new Json<graph>("airportsTest.json");
    auto read = new Json<graph>("airportsClean.json");
    auto graph = read->parseJson();

    cout << "Nodes -> " << graph.getNumberOfNodes() << endl;
    cout << "Edges -> " << graph.getNumberOfEdges() << endl;

    //newgraph->deleteNode("4027");
    //cout << "Del aeropuerto/node 4027" << endl;
    //cout << "Delete egde 4027 -> 3536 " << newgraph->deleteEdge("4027", "3536") << endl;
    //cout << "Delete egde 4024 -> 1767 " << newgraph->deleteEdge("4024", "1767") << endl;

    printf("Densidad: %4.7f\n", graph.getDensity() );

    auto mstPrimm = graph.prim("4027");
    auto mstKruskal = graph.kruskal();
    //mstKruskal->printGraph();
    //printf("Primm: Weight: %4.7f\n", mstPrimm->getEdgesWeightSum() );
    //printf("Kruskal: Weight: %4.7f\n", mstKruskal->getEdgesWeightSum() );

    //Tester::testPrimm();
    //Tester::testKruskal();

    return 0;
}
