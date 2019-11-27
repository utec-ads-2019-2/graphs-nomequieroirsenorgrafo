#include "Graph.h"
#include "../src/test/tester.h"

using namespace std;

int main() {


//    Tester::testnondirectedGraphToJSON("../data/graphToJsonNonDir.json");
//
//    Tester::testdiGraphToJSON("../data/graphToJsonDir.json");

//    Tester::testFromJson("../data/airportsTestWS.json");
//
//    Tester::testBipartiteFromJson("../data/bipartito.json");
//    Tester::testConnectedFromJson("../data/conexo.json");

    //Tester::testnondirectedGraphToJSON("../data/graphToJsonNonDir.json");

    //Tester::testdiGraphToJSON("../data/graphToJsonDir.json");

    Tester::testGraphcreation();
    Tester::testConnected();
    Tester::testBipartite();
//    Tester::testPrimm();
//    Tester::testKruskal();
//    Tester::testStronglyConnected();

//    Tester::testDfs();
//    Tester::testBellmanFord();
//    Tester::testBellmanFordfromJSON("../data/airports2710.json");
//    Tester::testDijkstra();
    Tester::testDijkstrafromJSON("../data/airports2710.json");
    Tester::testDijkstrafromJSONOnlyStart("../data/airports2710.json");
    //Tester::testBipartiteFromJson("../data/bipartito.json");
    //Tester::testConnectedFromJson("../data/conexo.json");

    //Tester::testPrimm();
    //Tester::testKruskal();
    //Tester::testStronglyConnected();
    Tester::testBellmanFordfromJSON("../data/airports2710.json");
    Tester::testBellmanFord2();
    Tester::testAstar();
    Tester::testAstar_2();
    Tester::testAstarfromJSON("../data/airports2710.json", "../data/astartsp.json");

    return 0;
}
