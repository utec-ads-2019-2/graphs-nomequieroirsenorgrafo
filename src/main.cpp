#include "Graph.h"
#include "Json.h"
#include "src/test/tester.h"

using namespace std;

int main() {

    Tester::testnondirectedGraphToJSON("../data/graphToJsonNonDir.json");

    Tester::testdiGraphToJSON("../data/graphToJsonDir.json");

    //Tester::testFromJson("../data/airportsTestWS.json");

    //Tester::testBipartiteFromJson("../data/bipartito.json");
    //Tester::testConnectedFromJson("../data/conexo.json");

    Tester::testGraphcreation();
    Tester::testConnected();
    Tester::testBipartite();
    Tester::testPrimm();
    Tester::testKruskal();
    Tester::testStronglyConnected();

//    delete json1;

    return 0;
}
