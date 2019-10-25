#include "tester.h"

void Tester::test_bfs() {
    graph testBfs;

    testBfs.addNode("v", 0, 0);
    testBfs.addNode("r", 0, 1);
    testBfs.addNode("s", 1, 1);
    testBfs.addNode("w", 1, 0);
    testBfs.addNode("x", 2, 0);
    testBfs.addNode("t", 2, 1);
    testBfs.addNode("y", 3, 0);
    testBfs.addNode("u", 3, 1);

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

    testBfs.bfs("s");

    for (int i=0 ; i < 8 ; i++) {
        exp->generateFromInfixExp(equations[i]);
        exp->askvalueVariables();
        ASSERT(exp->evaluate() == results[i], "Wrong result in ", equations[i]);
    }

}
