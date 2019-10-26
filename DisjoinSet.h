// Created by felix on 10/25/19.
#ifndef GRAPHS_NOMEQUIEROIRSENORGRAFO_DISJOINSET_H
#define GRAPHS_NOMEQUIEROIRSENORGRAFO_DISJOINSET_H

#include <bits/stdc++.h>
using namespace std;

template <typename T>
class DisjointSet {
    unordered_map<T, T> parent; // < Key, parent >
public:
    DisjointSet() = default;

    T& operator[](T i){ return parent[i]; }

    void makeSet(T element){ parent[element] = element; }

    T findSet(T element) {
        if (parent[element] == element) { return element; }
        return findSet(parent[element]);
    }

    void makeUnion(T set1, T set2) {
        T x = findSet(set1);
        T y = findSet(set2);
        parent[x] = parent[y];
    }
};


#endif //GRAPHS_NOMEQUIEROIRSENORGRAFO_DISJOINSET_H
