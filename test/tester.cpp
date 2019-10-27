#include "tester.h"

void Tester::testConnected() {
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
    auto connected = testBfs.isConnected()?"Yes":"No";
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

    auto connected2 = testBfs2.isConnected()?"Si":"No";
    cout<< "Graph 2 Is connected? -> "<<connected2<< endl;
    /*for (int i=0 ; i < 8 ; i++) {
        exp->generateFromInfixExp(equations[i]);
        exp->askvalueVariables();
        ASSERT(exp->evaluate() == results[i], "Wrong result in ", equations[i]);
    }*/

}

void Tester::testBipartite() {
    auto test1 = new graph();
    cout << "Test bipartite" << endl;
    test1->addVertex("a");
    test1->addVertex("b");
    test1->addVertex("c");
    test1->addVertex("d");
    test1->addVertex("e");
    test1->addVertex("f");

    test1->addEdge("a", "b", 1);
    test1->addEdge("b", "c", 1);
    test1->addEdge("c", "d", 1);
    test1->addEdge("d", "e", 1);
    test1->addEdge("e", "f", 1);
    test1->addEdge("f", "a", 1);

    test1->printGraph();
    auto bipartite = test1->isBipartite()?"Yes":"No";
    cout<< "is bipartite? " << bipartite << endl;
    cout << endl;

    auto test2 = new graph();
    test2->addVertex("a");
    test2->addVertex("b");
    test2->addVertex("c");
    test2->addVertex("d");
    test2->addVertex("e");

    test2->addEdge("a", "b", 1);
    test2->addEdge("b", "c", 1);
    test2->addEdge("c", "d", 1);
    test2->addEdge("d", "e", 1);
    test2->addEdge("e", "a", 1);
    test2->printGraph();
    bipartite = test2->isBipartite()?"Yes":"No";
    cout<< "is bipartite? " << bipartite << endl;
    cout << endl;
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

    cormen.addVertex("a"); cormen.addVertex("b"); cormen.addVertex("c");
    cormen.addVertex("d"); cormen.addVertex("e"); cormen.addVertex("f");
    cormen.addVertex("g"); cormen.addVertex("h"); cormen.addVertex("i");

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