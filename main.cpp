#include "Graph.h"
#include "Json.h"

using namespace std;

int main() {
    //graph test;

    //auto read = new Json<graph>("airportsTest.json");
    auto read = new Json<graph>("airports.json");

    read->parseJson();

    return 0;
}


