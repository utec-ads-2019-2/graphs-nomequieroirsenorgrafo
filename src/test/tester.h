#ifndef TESTER_H
#define TESTER_H

#include <stdexcept>
#include <iostream>
#include <cassert>
#include "../src/Graph.h"

using namespace std;

#ifndef NDEBUG
#   define ASSERT(condition, message1, message2) \
    do { \
        if (! (condition)) { \
            std::cerr << "Assertion `" #condition "` failed in " << __FILE__ \
                      << " line " << __LINE__ << ": " << message1 << " " << message2 << std::endl; \
            std::terminate(); \
        } \
    } while (false)
#else
#   define ASSERT(condition, message) do { } while (false)
#endif

class Tester {
private:
    static graph CormmenExample();

public:
    static void testGraphcreation();

    static void testConnected();

    static void testBipartite();

    static void testPrimm();

    static void testKruskal();

    static void testFromJson(string fileName);

    static void testBipartiteFromJson(string fileName);

    static void testConnectedFromJson(string fileName);

    static void testStronglyConnected();

    static void testdiGraphToJSON(string fileName);

    static void testnondirectedGraphToJSON(string fileName);

    static void testDfs();

    static void testAstar();

    static void testAstar_2();

    static void testAstarfromJSON(string filenameIn, string filenameOut);

    static void testBellmanFord();

    static void testBellmanFord2();

    static void testBellmanFordfromJSON(string fileName);

    static void testDijkstra();

    static void testDijkstrafromJSON(const string&);

    static void testDijkstrafromJSONOnlyStart(const string&);
};

#endif