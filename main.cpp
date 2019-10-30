#include "Graph.h"
#include "Json.h"
#include "test/tester.h"

using namespace std;

int main() {

    auto json1 = new Json<graph>("cmake-build-debug/airportsTestWS.json");
    auto graphTest2 = json1->parseJson();

    graphTest2.printGraph(); cout << endl;
    auto primGraphTest2 = graphTest2.prim("1229");
    primGraphTest2->printGraph();

    cout << "JSON" << endl;
    cout << "Nodes -> " << graphTest2.getNumberOfNodes() << endl;
    cout << "Edges -> " << graphTest2.getNumberOfEdges() << endl;
    printf("Densidad: %4.7f\n", graphTest2.getDensity() );
    auto connected = graphTest2.isConnected() ? "Yes" : "No";
    cout << "is connected? " << connected << endl;
    auto bipartite = graphTest2.isBipartite() ? "Yes" : "No";
    cout << "is bipartite? "<< bipartite << endl;
    auto stronglyConnected = graphTest2.isStronglyConnected() ? "Yes" : "No";
    cout << "is strongly connected? " << stronglyConnected << endl;

    /*auto mstPrimm = graphTest2.prim("4027");
    auto mstKruskal = graphTest2.kruskal();
    //mstKruskal->printGraph();
    printf("Primm: Weight: %4.7f\n", mstPrimm->getEdgesWeightSum() );
    printf("Kruskal: Weight: %4.7f\n", mstKruskal->getEdgesWeightSum() );

    auto json2 = new Json<graph>("bipartito.json");
    graphTest2 = json2->parseJson();

    cout << ">>JSON bipartito.json<<" << endl;
    cout << "Nodes -> " << graphTest2.getNumberOfNodes() << endl;
    cout << "Edges -> " << graphTest2.getNumberOfEdges() << endl;
    printf("Densidad: %4.7f\n", graphTest2.getDensity() );
    connected = graphTest2.isConnected() ? "Yes" : "No";
    cout << "is connected? " << connected << endl;
    bipartite = graphTest2.isBipartite() ? "Yes" : "No";
    cout << "is bipartite? "<< bipartite << endl;

    auto json3 = new Json<graph>("conexo.json");
    graphTest2 = json3->parseJson();

    cout << ">>JSON conexo.json<<" << endl;
    cout << "Nodes -> " << graphTest2.getNumberOfNodes() << endl;
    cout << "Edges -> " << graphTest2.getNumberOfEdges() << endl;
    printf("Densidad: %4.7f\n", graphTest2.getDensity() );
    connected = graphTest2.isConnected() ? "Yes" : "No";
    cout << "is connected? " << connected << endl;
    bipartite = graphTest2.isBipartite() ? "Yes" : "No";
    cout << "is bipartite? "<< bipartite << endl;*/

    /*Tester::testGraphcreation();
    Tester::testConnected();
    Tester::testBipartite();*/
    Tester::testPrimm();
    //Tester::testKruskal();

//    delete json1;

    return 0;
}
