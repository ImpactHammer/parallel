

#include <limits.h>
#include <pthread.h>
#include <iostream>
#include <fstream>
#include <unistd.h>
#include <time.h>

#include "graph.h"


struct timespec start, stop;
double elapsed;
bool optimize;


std::vector<int> shortest_path(graph g, int src, int dst) {
    Nodes nodes = g.nodes;
    Edges edges = g.edges;
    int* parent;
    int* estim;
    int n_nodes = g.n_nodes;
    
    parent = new int [n_nodes];
    estim = new int [n_nodes];
    
    for (Edges::iterator it_edges = edges.begin(); it_edges != edges.end(); it_edges++) {
        int i_node = it_edges->first;
        estim[i_node] = INT_MAX / 2;
        parent[i_node] = -1;
    }
    estim[src] = 0;
    
    bool parity = false;
    bool early_stopping = true;
    
    for (int i_pass = 0; i_pass < nodes.size(); i_pass++) {
        for (int i_node = 0; i_node < nodes.size(); i_node++) {
            int node = nodes[i_node];
            for (Rays::iterator it_ray = edges[node].begin();
                 it_ray != edges[node].end(); it_ray++) {
                int src = it_ray->first;
                int weight = it_ray->second;
                int len_upd = estim[src] + weight;
                if (len_upd < estim[node]) {
                    early_stopping = false;
                    estim[node] = len_upd;
                    parent[node] = src;
                }
            }
        }
        if (early_stopping && optimize)
            break;
        early_stopping = true;
    }
    if (estim[dst] == INT_MAX / 2) {
        return std::vector<int>();
    } else {
        std::vector<int> path;
        int elem_cur = dst;
        while (true) {
            path.push_back(elem_cur);
            if (elem_cur == src)
                break;
            elem_cur = parent[elem_cur];
        }
        std::reverse(path.begin(), path.end());
        return path;
    }
}


int main(int argc, char** argv) {
    graph g;
    g.read("graph");
    
    if (argc > 1) {
        optimize = (bool) atoi(argv[1]);
    } else {
        optimize = false;
    }
    
    int src = 0;
    int dst = 1;
    
    struct timespec start, stop;
    double elapsed;
    //~ while (true) {
        clock_gettime(CLOCK_MONOTONIC, &start);
        std::vector<int> path = shortest_path(g, src, dst);
        clock_gettime(CLOCK_MONOTONIC, &stop);
        elapsed = (stop.tv_sec - start.tv_sec);
        elapsed += (stop.tv_nsec - start.tv_nsec) / 1000000000.0;
        for (std::vector<int>::iterator it = path.begin(); it != path.end(); it++) {
            printf("%i ", *it);
        }
        printf("\n");
        printf("%lf\n", elapsed);
    //~ }
	return 0;
}
