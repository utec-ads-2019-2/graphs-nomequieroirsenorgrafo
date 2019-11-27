#include "tester.h"
#include "../src/Json.h"

void Tester::testStronglyConnected() {
    auto test1 = new graph(true);

    // CORMEN DIRECTED GRAPH
    test1->addVertex("q", 0, 0);
    test1->addVertex("r", 0, 0);
    test1->addVertex("s", 0, 0);
    test1->addVertex("t", 0, 0);
    test1->addVertex("u", 0, 0);
    test1->addVertex("v", 0, 0);
    test1->addVertex("w", 0, 0);
    test1->addVertex("x", 0, 0);
    test1->addVertex("y", 0, 0);
    test1->addVertex("z", 0, 0);

    test1->addEdge("r", "u", 1);
    test1->addEdge("r", "y", 1);
    test1->addEdge("u", "y", 1);
    test1->addEdge("y", "q", 1);
    test1->addEdge("t", "x", 1);
    test1->addEdge("t", "y", 1);
    test1->addEdge("x", "z", 1);
    test1->addEdge("z", "x", 1);
    test1->addEdge("q", "s", 1);
    test1->addEdge("q", "w", 1);
    test1->addEdge("q", "t", 1);
    test1->addEdge("s", "v", 1);
    test1->addEdge("v", "w", 1);
    test1->addEdge("w", "s", 1);

//    test1->printGraph();
    auto msg2 = "is strongly connected graph?";
    ASSERT(!test1->isStronglyConnected(), "Wrong result in ", msg2);
    cout << "PASS STRONGLY CONNECTED"<< endl;

    delete test1;

    // TEST JSON
    auto json1 = new Json<graph>("../data/airports.json");
    auto test2 = json1->parseJson();
    auto msg3 = "is strongly connected graph?";

    ASSERT(!test2.isStronglyConnected(), "Wrong result in ", msg3);
    cout << "PASS STRONGLY CONNECTED"<< endl;
}


void Tester::testConnected() {
    graph test1;
    cout << "Test for connected graph" << endl;
    test1.addVertex("v", 0, 0);
    test1.addVertex("r", 0, 1);
    test1.addVertex("s", 1, 1);
    test1.addVertex("w", 1, 0);
    test1.addVertex("x", 2, 0);
    test1.addVertex("t", 2, 1);
    test1.addVertex("y", 3, 0);
    test1.addVertex("u", 3, 1);

    test1.addEdge("v", "r", 0);
    test1.addEdge("r", "s", 0);
    test1.addEdge("w", "x", 0);
    test1.addEdge("s", "w", 0);
    test1.addEdge("w", "t", 0);
    test1.addEdge("x", "t", 0);
    test1.addEdge("x", "u", 0);
    test1.addEdge("x", "y", 0);
    test1.addEdge("t", "u", 0);
    test1.addEdge("u", "y", 0);

    //test1.printGraph();
    //test1.bfs("s");
    auto msg2 = "is connected graph?";
    ASSERT(test1.isConnected(), "Wrong result in ", msg2);

    graph test2;

    test2.addVertex("9", 0, 0);
    test2.addVertex("26", 0, 0);
    test2.addVertex("11", 0, 0);
    test2.addVertex("18", 0, 0);
    test2.addVertex("19", 0, 0);
    test2.addVertex("13", 0, 0);
    test2.addVertex("5", 0, 0);
    test2.addVertex("17", 0, 0);
    test2.addVertex("23", 0, 0);
    test2.addVertex("24", 0, 0);

    test2.addEdge("9", "26", 0);
    test2.addEdge("9", "19", 0);
    test2.addEdge("9", "18", 0);
    test2.addEdge("26", "11", 0);
    test2.addEdge("9", "19", 0);
    test2.addEdge("13", "19", 0);
    test2.addEdge("13", "5", 0);
    test2.addEdge("23", "24", 0);

    ASSERT(!test2.isConnected(), "Wrong result in ", msg2);

}

void Tester::testBipartite() {
    auto test1 = new graph();
    cout << "Test bipartite" << endl;
    test1->addVertex("a", 0, 0);
    test1->addVertex("b", 0, 0);
    test1->addVertex("c", 0, 0);
    test1->addVertex("d", 0, 0);
    test1->addVertex("e", 0, 0);
    test1->addVertex("f", 0, 0);

    test1->addEdge("a", "b", 1);
    test1->addEdge("b", "c", 1);
    test1->addEdge("c", "d", 1);
    test1->addEdge("d", "e", 1);
    test1->addEdge("e", "f", 1);
    test1->addEdge("f", "a", 1);

    //test1->printGraph();
    auto msg2 = "case 1: is bipartite?";
    ASSERT(test1->isBipartite(), "Wrong result in ", msg2);
    cout << "case 1 all ok"<< endl;

    delete test1;

    auto test2 = new graph();
    test2->addVertex("a", 0, 0);
    test2->addVertex("b", 0, 0);
    test2->addVertex("c", 0, 0);
    test2->addVertex("d", 0, 0);
    test2->addVertex("e", 0, 0);

    test2->addEdge("a", "b", 1);
    test2->addEdge("b", "c", 1);
    test2->addEdge("c", "d", 1);
    test2->addEdge("d", "e", 1);
    test2->addEdge("e", "a", 1);
    //test2->printGraph();

    msg2 = "case 2: is bipartite?";
    ASSERT(!test2->isBipartite(), "Wrong result in ", msg2);
    cout << "case 2 all ok"<< endl;
    delete test2;
}

void Tester::testPrimm() {
    Traits::E totalWeight = 93;
    Traits::E mstWeight = 37;

    auto cormen = CormmenExample();

    cout << "Testing Primm MST" << endl;
    cout << cormen.getNumberOfNodes() << " " << cormen.getNumberOfEdges() << endl;

    cormen.printGraph();
    cout << endl;
    //Sum of edges weight
    auto msg2 = "Sum of edges weight";
    ASSERT(cormen.getEdgesWeightSum() == totalWeight, "Wrong result in ",
            msg2);

    auto primCormen = cormen.prim("a");
    primCormen->printGraph();
    msg2 = "MST weight";
    ASSERT(primCormen->getEdgesWeightSum() == mstWeight, "Wrong result in ",
           msg2);
    //printf("MST with weight: %4.7f", primCormen->getEdgesWeightSum() );

    cout << "PASS" << endl;
}

void Tester::testKruskal() {
    Traits::E totalWeight = 93;
    Traits::E mstWeight = 37;

    auto cormen = CormmenExample();

    cout << "Testing Kruskal MST" << endl;
    cout << cormen.getNumberOfNodes() << " " << cormen.getNumberOfEdges() << endl;
    cormen.printGraph();

    cout << endl;
    //Sum of edges weight
    auto msg2 = "Sum of edges weight";
    ASSERT(cormen.getEdgesWeightSum() == totalWeight, "Wrong result in ",
           msg2);

    auto kruskalCormen = cormen.kruskal();
    kruskalCormen->printGraph();
    msg2 = "MST weight";
    ASSERT(kruskalCormen->getEdgesWeightSum() == mstWeight, "Wrong result in ",
           msg2);
    //printf("MST with weight: %4.7f", primCormen->getEdgesWeightSum() );

    cout << "PASS" << endl;
}

graph Tester::CormmenExample() {
    graph cormen;

    cormen.addVertex("a", 0, 0); cormen.addVertex("b", 0, 0); cormen.addVertex("c", 0, 0);
    cormen.addVertex("d", 0, 0); cormen.addVertex("e", 0, 0); cormen.addVertex("f", 0, 0);
    cormen.addVertex("g", 0, 0); cormen.addVertex("h", 0, 0); cormen.addVertex("i", 0, 0);

    cormen.addEdge("a", "b", 4);
    cormen.addEdge("a", "h", 8);
    cormen.addEdge("b", "c", 8);
    cormen.addEdge("b", "h", 11);
    cormen.addEdge("c", "d", 7);
    cormen.addEdge("c", "i", 2);
    cormen.addEdge("c", "f", 4);
    cormen.addEdge("d", "e", 9);
    cormen.addEdge("d", "f", 14);
    cormen.addEdge("e", "f", 10);
    cormen.addEdge("f", "g", 2);
    cormen.addEdge("g", "h", 1);
    cormen.addEdge("g", "i", 6);
    cormen.addEdge("h", "i", 7);

    return cormen;
}

void Tester::testGraphcreation() {
    int cantVertex = 6;
    int cantEdges = 7;
    auto test1 = new graph();
//  Non directed graph
    cout<< "Test for non directed graph" << endl;
    test1->addVertex("a", 0, 4);
    test1->addVertex("b", 3, 6);
    test1->addVertex("c", 6, 4);
    test1->addVertex("d", 6, 2);
    test1->addVertex("e", 3, 0);
    test1->addVertex("f", 0, 2);

    auto msg2 = "Adding vertex";
    ASSERT(test1->getNumberOfNodes() == cantVertex, "Wrong result in ",
           msg2);

    test1->addEdge("a", "b", 0);
    test1->addEdge("b", "c", 0);
    test1->addEdge("c", "d", 0);
    test1->addEdge("d", "e", 0);
    test1->addEdge("e", "f", 0);
    test1->addEdge("f", "a", 0);
    test1->addEdge("d", "a", 0);

    msg2 = "Adding edges";
    ASSERT(test1->getNumberOfEdges() == cantEdges, "Wrong result in ",
           msg2);

    test1->deleteNode("a");
    test1->deleteNode("c");

    msg2 = "Deleting 2 vertex";
    cantVertex = cantVertex - 2;
    ASSERT(test1->getNumberOfNodes() == cantVertex, "Wrong result in ",
           msg2);

    msg2 = "Edges after delete 2 vertex";
    cantEdges = cantEdges - 5; //3(a) + 2(c)
    ASSERT(test1->getNumberOfEdges() == cantEdges, "Wrong result in ",
           msg2);

    test1->deleteEdge("e", "f");
    --cantEdges;
    msg2 = "Edges after delete e-f edge";
    ASSERT(test1->getNumberOfEdges() == cantEdges, "Wrong result in ",
           msg2);

    auto result = test1->findvertex("a");

    msg2 = "Find deleted vertex a";
    ASSERT(!result, "Wrong result in ", msg2);

    result = test1->findvertex("b");

    msg2 = "Find deleted vertex b";
    ASSERT(result, "Wrong result in ", msg2);

    result = test1->findvertex("c");

    msg2 = "Find deleted vertex c";
    ASSERT(!result, "Wrong result in ", msg2);
    cout << "all Ok\n" <<endl;

//  Directed graph
    cout<< "Test for directed graph" << endl;
    test1 = new graph(true);
    test1->addVertex("q", 0, 0);
    test1->addVertex("s", 0, 0);
    test1->addVertex("v", 0, 0);
    test1->addVertex("w", 0, 0);
    test1->addVertex("t", 0, 0);
    test1->addVertex("x", 0, 0);
    test1->addVertex("z", 0, 0);
    test1->addVertex("y", 0, 0);
    test1->addVertex("r", 0, 0);
    test1->addVertex("u", 0, 0);

    cantVertex = 10;
    msg2 = "Adding vertex";
    ASSERT(test1->getNumberOfNodes() == cantVertex, "Wrong result in ",
           msg2);

    test1->addEdge("q","t",3);
    test1->addEdge("q","s",3);
    test1->addEdge("q","w",3);
    test1->addEdge("s","v",2);
    test1->addEdge("v","w",2);
    test1->addEdge("w","s",2);
    test1->addEdge("t","x",2);
    test1->addEdge("t","y",2);
    test1->addEdge("x","z",2);
    test1->addEdge("z","x",2);
    test1->addEdge("y","q",2);
    test1->addEdge("r","u",3);
    test1->addEdge("r","y",3);
    test1->addEdge("u","y",3);
    cantEdges = 14;

    msg2 = "Adding edges";
    ASSERT(test1->getNumberOfEdges() == cantEdges, "Wrong result in ",
           msg2);

    test1->deleteNode("s");
    test1->deleteNode("w");
    test1->deleteNode("v");
    msg2 = "Deleting 3 vertex";
    cantVertex = cantVertex - 3;
    ASSERT(test1->getNumberOfNodes() == cantVertex, "Wrong result in ",
           msg2);

    msg2 = "Edges after delete 3 vertex";
    cantEdges = cantEdges - 5;
    ASSERT(test1->getNumberOfEdges() == cantEdges, "Wrong result in ",
           msg2);

    test1->deleteEdge("y", "q");
    --cantEdges;
    msg2 = "Edges after delete y-q edge";
    ASSERT(test1->getNumberOfEdges() == cantEdges, "Wrong result in ",
           msg2);

    result = test1->findvertex("w");

    msg2 = "Find deleted vertex w";
    ASSERT(!result, "Wrong result in ", msg2);
    cout << "all ok\n" << endl;

    delete test1;
}

void Tester::testBipartiteFromJson(string fileName) {
    auto json12 = new Json<graph>(fileName);
    auto graphTest2 = json12->parseJson();

    cout << ">>JSON bipartito.json<<" << endl;
    cout << "Nodes -> " << graphTest2.getNumberOfNodes() << endl;
    cout << "Edges -> " << graphTest2.getNumberOfEdges() << endl;
    printf("Densidad: %4.7f\n", graphTest2.getDensity() );
    auto connected = graphTest2.isConnected() ? "Yes" : "No";
    cout << "is connected? " << connected << endl;
    auto bipartite = graphTest2.isBipartite() ? "Yes" : "No";
    cout << "is bipartite? "<< bipartite << endl;

    delete json12;
}

void Tester::testConnectedFromJson(string fileName) {
    auto json4 = new Json<graph>(fileName);
    auto graphTest2 = json4->parseJson();

    cout << ">>JSON conexo.json<<" << endl;
    cout << "Nodes -> " << graphTest2.getNumberOfNodes() << endl;
    cout << "Edges -> " << graphTest2.getNumberOfEdges() << endl;
    printf("Densidad: %4.7f\n", graphTest2.getDensity() );
    auto connected = graphTest2.isConnected() ? "Yes" : "No";
    cout << "is connected? " << connected << endl;
    auto bipartite = graphTest2.isBipartite() ? "Yes" : "No";
    cout << "is bipartite? "<< bipartite << endl;
}

void Tester::testdiGraphToJSON(string fileName) {
    //  Directed graph
    cout<< "Test for generate JSON from directed graph" << endl;
    //auto test2 = new graph(true); // doesn't work, call destructor
    auto test2 =  graph(true);
    test2.addVertex("q", 0, 0);
    test2.addVertex("s", 0, 0);
    test2.addVertex("v", 0, 0);
    test2.addVertex("w", 0, 0);
    test2.addVertex("t", 0, 0);
    test2.addVertex("x", 0, 0);
    test2.addVertex("z", 0, 0);
    test2.addVertex("y", 0, 0);
    test2.addVertex("r", 0, 0);
    test2.addVertex("u", 0, 0);

    test2.addEdge("q","t",3);
    test2.addEdge("q","s",3);
    test2.addEdge("q","w",3);
    test2.addEdge("s","v",2);
    test2.addEdge("v","w",2);
    test2.addEdge("w","s",2);
    test2.addEdge("t","x",2);
    test2.addEdge("t","y",2);
    test2.addEdge("x","z",2);
    test2.addEdge("z","x",2);
    test2.addEdge("y","q",2);
    test2.addEdge("r","u",3);
    test2.addEdge("r","y",3);
    test2.addEdge("u","y",3);

    auto json3 = new Json<graph>(fileName);
    json3->parseGraph(test2);
}

void Tester::testnondirectedGraphToJSON(string fileName) {
    //Graph to JSON: non directed
    cout<< "Test for generate JSON from non-directed graph" << endl;
    graph test1;
    test1.addVertex("v", 0, 0);
    test1.addVertex("r", 0, 1);
    test1.addVertex("s", 1, 1);
    test1.addVertex("w", 1, 0);
    test1.addVertex("x", 2, 0);
    test1.addVertex("t", 2, 1);
    test1.addVertex("y", 3, 0);
    test1.addVertex("u", 3, 1);

    test1.addEdge("v", "r", 0);
    test1.addEdge("r", "s", 0);
    test1.addEdge("w", "x", 0);
    test1.addEdge("s", "w", 0);
    test1.addEdge("w", "t", 0);
    test1.addEdge("x", "t", 0);
    test1.addEdge("x", "u", 0);
    test1.addEdge("x", "y", 0);
    test1.addEdge("t", "u", 0);
    test1.addEdge("u", "y", 0);

    auto json2 = new Json<graph>(fileName);
    json2->parseGraph(test1);
}

void Tester::testFromJson(string fileName) {
    auto json1 = new Json<graph>(fileName);
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
    cout << "is strongly connected? " << stronglyConnected << endl << endl;

    auto mstPrimm = graphTest2.prim("4027");
    auto mstKruskal = graphTest2.kruskal();
    //mstKruskal->printGraph();
    printf("Primm: Weight: %4.7f\n", mstPrimm->getEdgesWeightSum() );
    printf("Kruskal: Weight: %4.7f\n", mstKruskal->getEdgesWeightSum() );
}


void Tester::testDfs() {
    graph dfsGraph;
//    dfsGraph.addVertex("a"); dfsGraph.addVertex("b");
//    dfsGraph.addVertex("c"); dfsGraph.addVertex("d");
//
//    dfsGraph.addEdge("a", "b", 1);
//    dfsGraph.addEdge("b", "c", 2);
//    dfsGraph.addEdge("c", "d", 3);
//    dfsGraph.addEdge("d", "a", 4);
    dfsGraph.addVertex("0", 0, 0);
    dfsGraph.addVertex("1", 0, 0);
    dfsGraph.addVertex("2", 0, 0);
    dfsGraph.addVertex("3", 0, 0);
    dfsGraph.addVertex("4", 0, 0);
    dfsGraph.addVertex("5", 0, 0);
    dfsGraph.addVertex("6", 0, 0);
    dfsGraph.addVertex("7", 0, 0);

    dfsGraph.addEdge("0", "2", 1);

    dfsGraph.addEdge("1", "6", 2);

    dfsGraph.addEdge("2", "1", 1);
    dfsGraph.addEdge("2", "4", 1);
    dfsGraph.addEdge("2", "6", 1);

    dfsGraph.addEdge("6", "4", 1);
    dfsGraph.addEdge("6", "5", 5);
    dfsGraph.addEdge("6", "7", 1);

    dfsGraph.addEdge("5", "3", 3);
    dfsGraph.addEdge("5", "7", 1);

    auto dfs = dfsGraph.dfs("0");
//    for (auto & df : dfs){
//        auto v = df.first;
//        auto pairIntInt = df.second;
//        cout << "N = " << v << " Time In = " << pairIntInt.first << " Time Out = " << pairIntInt.second << endl;
//    }
    dfs.printGraph();
}

void Tester::testAstar() {
    graph test1(true);
    test1.addVertex("a", 1, 4);
    test1.addVertex("b", 5, 1);
    test1.addVertex("c", 5, 9);
    test1.addVertex("d", 8, 1);
    test1.addVertex("e", 8, 5);
    test1.addVertex("f", 11, 4);

    test1.addEdge("a", "b", 316);
    test1.addEdge("a", "c", 316);
    test1.addEdge("b", "d", 167);
    test1.addEdge("b", "e", 367);
    test1.addEdge("c", "d", 467);
    test1.addEdge("c", "e", 267);
    test1.addEdge("d", "e", 200);
    test1.addEdge("d", "f", 317);
    test1.addEdge("e", "f", 217);

    auto startNode = "a";
    auto targetNode = "f";
    cout << "Testing A* shorstest path from " <<startNode << " to "<<targetNode<<endl;
    auto aStar = test1.a_star_sp(startNode, targetNode);
    aStar.printGraph();
}

void Tester::testAstar_2() {
    graph test1(true);
    test1.addVertex("s", 1, 4);
    test1.addVertex("h", 3, 1);
    test1.addVertex("f", 5, 4);
    test1.addVertex("a", 3, 7);
    test1.addVertex("e", 7, 1);
    test1.addVertex("b", 5, 7);
    test1.addVertex("c", 7, 7);
    test1.addVertex("g", 10, 1);
    test1.addVertex("d", 12, 4);

    test1.addEdge("s","a", 5);
    test1.addEdge("s","h", 2);
    test1.addEdge("h","a", 2);
    test1.addEdge("a","f", 3);
    test1.addEdge("f","h", 2);
    test1.addEdge("a","b", 7);
    test1.addEdge("f","b", 2);
    test1.addEdge("f","e", 6);
    test1.addEdge("h","e", 9);
    test1.addEdge("b","e", 5);
    test1.addEdge("b","c", 8);
    test1.addEdge("b","g", 7);
    test1.addEdge("e","g", 2);
    test1.addEdge("g","c", 3);
    test1.addEdge("c","d", 4);

    auto startNode = "s";
    auto targetNode = "d";
    cout << "Testing A* shorstest path from " <<startNode << " to "<<targetNode<<endl;
    auto aStar = test1.a_star_sp(startNode, targetNode);
    aStar.printGraph();
}

void Tester::testAstarfromJSON(string filenameIn, string filenameOut) {
    auto jsonInput = new Json<graph>(filenameIn);
    auto graphTest = jsonInput->parseJson();
    //Lima: 2789 Vietnam: 3199
    auto startNode = "7252";
    auto targetNode = "3199"; //"6114";
    cout << "Testing A* shorstest path of " <<filenameIn<<" from airport "<< startNode <<
    " to airport "<< targetNode <<endl;

    auto aStar = graphTest.a_star_sp(startNode, targetNode);
    aStar.printGraph();

    auto jsonOutput = new Json<graph>(filenameOut);
    jsonOutput->parseGraph(aStar);
}

void Tester::testBellmanFord() {
    graph Graph(true);
    Graph.addVertex("a", 2, 0); Graph.addVertex("b", 3, 0); Graph.addVertex("c", 6, 0);
    Graph.addVertex("d", 0, 2); Graph.addVertex("e", 0, 5); Graph.addVertex("f", 0, 7);

    Graph.addEdge("a", "b", 10);

    Graph.addEdge("b", "c", 1);

    Graph.addEdge("c", "e", 3);

    Graph.addEdge("e", "f", 22);
    Graph.addEdge("e", "d", -10);

    Graph.addEdge("d", "b", 4);

    auto bellman = Graph.bellmanFord("a");

    cout << std::boolalpha << bellman.first << endl;

    bellman.second.printGraph();

}

void Tester::testBellmanFord2() {
    graph test1(true);
    test1.addVertex("s", 1, 4);
    test1.addVertex("h", 3, 1);
    test1.addVertex("f", 5, 4);
    test1.addVertex("a", 3, 7);
    test1.addVertex("e", 7, 1);
    test1.addVertex("b", 5, 7);
    test1.addVertex("c", 7, 7);
    test1.addVertex("g", 10, 1);
    test1.addVertex("d", 12, 4);

    test1.addDirectedEdge("s","a"/*, 5*/);
    test1.addDirectedEdge("s","h"/*, 2*/);
    test1.addDirectedEdge("h","a"/*, 2*/);
    test1.addDirectedEdge("a","f"/*, 3*/);
    test1.addDirectedEdge("f","h"/*, -2*/);
    test1.addDirectedEdge("a","b"/*, 7*/);
    test1.addDirectedEdge("f","b"/*, 2*/);
    test1.addDirectedEdge("f","e"/*, 6*/);
    test1.addDirectedEdge("h","e"/*, 9*/);
    test1.addDirectedEdge("b","e"/*, 5*/);
    test1.addDirectedEdge("b","c"/*, 8*/);
    test1.addDirectedEdge("b","g"/*, 7*/);
    test1.addDirectedEdge("e","g"/*, 2*/);
    test1.addDirectedEdge("g","c"/*, 3*/);
    test1.addDirectedEdge("c","d"/*, 4*/);

    auto startNode = "s";

    cout << "Testing Bellman Ford shorstest path from " <<startNode<<endl;
    auto bellmanResult = test1.bellmanFord(startNode);
    auto bellmanBool = bellmanResult.first ? "Yes" : "No";
    cout << "Result ? " << bellmanBool << endl;
    bellmanResult.second.printGraph();
}

void Tester::testBellmanFordfromJSON(string fileName) {
    auto jsonInput = new Json<graph>(fileName);
    auto graphTest = jsonInput->parseJson();

    auto startNode = "7252";
    //auto targetNode = "2789"; //"3199";
    cout << "Testing Bellman Ford shorstest path of " <<fileName<<" from airport "<< startNode <<endl;

    auto bellmanResult = graphTest.bellmanFord(startNode);
    auto bellmanBool = bellmanResult.first ? "Yes" : "No";
    cout << "Result ? " << bellmanBool << endl;

    auto jsonOutput = new Json<graph>("../data/bellmanford.json");
    jsonOutput->parseGraph(bellmanResult.second);
    cout << "Nodes -> " << bellmanResult.second.getNumberOfNodes() << " Edges -> "
        << bellmanResult.second.getNumberOfEdges() << endl;
}

void Tester::testDijkstra() {
    graph Graph(true);
    Graph.addVertex("0", 0, 0); Graph.addVertex("1", 0, 0); Graph.addVertex("2", 0, 0);
    Graph.addVertex("3", 0, 0); Graph.addVertex("4", 0, 0); Graph.addVertex("5", 0, 0);
    Graph.addVertex("6", 0, 0); Graph.addVertex("7", 0, 0);

    Graph.addEdge("0", "1", 8);
    Graph.addEdge("0", "3", 2);

    Graph.addEdge("3", "7", 9);

    Graph.addEdge("4", "2", 3);

    Graph.addEdge("5", "1", 2);

    Graph.addEdge("6", "4", 9);

    Graph.addEdge("7", "5", 5);
    Graph.addEdge("7", "6", 6);

    auto dijkstra = Graph.dijkstra("0"); // Testing dijkstra only start on a little graph
    dijkstra.printGraph();
//    auto dijkstra = Graph.dijkstra("0", "3");

//    dijkstra.printGraph();

//    for (auto && item: dijkstra) {
//        cout << item.first << endl;
//        item.second->printGraph();
//    }
//    for (auto && item: dijkstra){
//        cout << item.first << endl;
//        item.second->printGraph();
//    }
}

void Tester::testDijkstrafromJSON(const string& fileName) {
    auto jsonInput = new Json<graph>(fileName);
    auto graphTest = jsonInput->parseJson();

    auto startNode = "7252";
    auto targetNode = "3199"; // "2789";
    cout << "Testing Dijkstra shorstest path of " <<fileName<<" from airport "<< startNode <<
         " to airport"<< targetNode <<endl;

    auto dijkstra = graphTest.dijkstra(startNode, targetNode);
    dijkstra.printGraph();

    auto jsonOutput = new Json<graph>("../data/dijkstrasp.json");
    jsonOutput->parseGraph(dijkstra);
}

void Tester::testDijkstrafromJSONOnlyStart(const string& fileName){
    auto jsonInput = new Json<graph>(fileName);
    auto graphTest = jsonInput->parseJson();

    auto startNode = "7252";
//    auto targetNode = "2789"; //"3199";
    cout << "Testing Dijkstra shorstest path of " <<fileName<<" from airport "<< startNode <<endl;

    auto dijkstra = graphTest.dijkstra(startNode);
//    dijkstra.printGraph();

    auto jsonOutput = new Json<graph>("../data/dijkstraspOnlyStart.json");
    jsonOutput->parseGraph(dijkstra);
    cout << "Nodes -> " << dijkstra.getNumberOfNodes() << " Edges -> "
         << dijkstra.getNumberOfEdges() << endl;
}