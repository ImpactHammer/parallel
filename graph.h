
#include <math.h>
#include <vector>
#include <map>
#include <unordered_map>
#include <set>
#include <algorithm>

typedef int Node;
typedef int Weight;
typedef std::vector<Node> Nodes;
typedef std::unordered_map<Node, Weight> Rays;
typedef std::unordered_map<Node, Rays> Edges;

struct graph {
    Nodes nodes;
    Edges edges;
    int n_nodes;
    
    void read(const char* fname) {
        int src;
        int dst;
        int weight;
        std::ifstream fin;
        fin.open(fname);
        n_nodes = 0;
        while (fin >> src >> dst >> weight) {
            if (src > n_nodes) {
                n_nodes = src;
            }
            if (dst > n_nodes) {
                n_nodes = dst;
            }
            edges[dst][src] = weight;
            if (edges.find(src) == edges.end()) {
                edges[src] = Rays();
            }
        }
        n_nodes++;
        for (Edges::iterator it = edges.begin(); it != edges.end(); it++) {
            nodes.push_back(it->first);
        }
    }
};
