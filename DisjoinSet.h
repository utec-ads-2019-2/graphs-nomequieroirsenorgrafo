// Created by felix on 10/25/19.
#ifndef GRAPHS_NOMEQUIEROIRSENORGRAFO_DISJOINSET_H
#define GRAPHS_NOMEQUIEROIRSENORGRAFO_DISJOINSET_H

#include <bits/stdc++.h>
using namespace std;

template <typename T>
class DisjointSet {
    unordered_map<T, T> parent;
public:
    DisjointSet() = default;

    T& operator[](T i){ return parent[i]; }

    void makeSet(std::vector<T> const &universe){
        for (int i : universe)
            parent[i] = i;
    }
    T Find(T element) {
        if (parent[element] == element) return element;
        return Find(parent[element]);
    }
    void makeUnion(T set1, T set2) {
        T x = Find(set1);
        T y = Find(set2);
        parent[x] = parent[y];
    }

    bool areInSameSet(T a, T b){ return find(a) == find(b); }
};


#endif //GRAPHS_NOMEQUIEROIRSENORGRAFO_DISJOINSET_H
