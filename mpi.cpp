
#include <sys/shm.h>
#include <limits.h>
#include <mpi.h>
#include <pthread.h>
#include <iostream>
#include <fstream>
#include <unistd.h>
#include <time.h>

#include "graph.h"

#define RANK_ROOT 0

#include <errno.h>
extern int errno;


struct timespec start, stop;
double elapsed;

bool optimize;


pthread_mutex_t mutex = PTHREAD_MUTEX_INITIALIZER;
pthread_cond_t cond = PTHREAD_COND_INITIALIZER;

struct thread_arg {
    int from;
    int to;
    bool* parity;
    bool* early_stopping;
    bool* waiting;
    bool* all_done;
    Nodes* nodes;
    Edges* edges;
    int** parent;
    int** estim;
};

int rank;
int n_threads;

void* bf_thread(void* arg) {
    

clock_gettime(CLOCK_MONOTONIC, &start);
    
    struct thread_arg* thr_arg = (struct thread_arg*) arg;
    int from = thr_arg->from;
    int to = thr_arg->to;
    bool& parity = *(thr_arg->parity);
    bool& waiting = *(thr_arg->waiting);
    bool& all_done = *(thr_arg->all_done);
    bool& early_stopping = *(thr_arg->early_stopping);
    Edges& edges = *(thr_arg->edges);
    Nodes& nodes = *(thr_arg->nodes);
    int** parent = thr_arg->parent;
    int** estim = thr_arg->estim;
    
    int cnt = 0;
    for (int i_node = from; i_node < to; i_node++) {
        int node = i_node;
        for (Rays::iterator it_ray = edges[node].begin();
             it_ray != edges[node].end(); it_ray++) {
                 cnt++;
             int src = it_ray->first;
             int weight = it_ray->second;
             int len_upd = estim[!parity][src] + weight;
             if (len_upd < estim[parity][node]) {
                 early_stopping = false;
                 estim[parity][node] = len_upd;
                 parent[parity][node] = src;
             } else if (estim[!parity][node] < estim[parity][node]) {
                 estim[parity][node] = estim[!parity][node];
                 parent[parity][node] = parent[!parity][node];
             }
        }
    }
    waiting = true;
    
}


std::vector<int> shortest_path_pthread(graph g, int src, int dst) {
    MPI_Comm_rank(MPI_COMM_WORLD, &rank);
    MPI_Comm_size(MPI_COMM_WORLD, &n_threads);
    
    Nodes nodes = g.nodes;
    Edges edges = g.edges;
    int* parent[2];
    int* estim[2];
    int n_nodes = g.n_nodes;
    
    int* displs = new int [n_threads];
    int* recvcounts = new int [n_threads];
    displs[0] = 0;
    int nodes_passed = 0;
    int rays_passed = 0;
    int i_start = 1;
    
    int n_edges = 0;
    for (Edges::iterator it_edges = edges.begin(); it_edges != edges.end(); it_edges++) {
        Rays& rays = it_edges->second;
        for (Rays::iterator it_rays = rays.begin(); it_rays != rays.end(); it_rays++) {
            n_edges++;
        }
    }
    int batch_size = n_edges / n_threads;
    
    for (Edges::iterator it_edges = edges.begin(); it_edges != edges.end(); it_edges++) {
        Rays& rays = it_edges->second;
        for (Rays::iterator it_rays = rays.begin(); it_rays != rays.end(); it_rays++) {
            rays_passed++;
        }
        while (rays_passed >= batch_size) {
            rays_passed -= batch_size;
            if (i_start < n_threads) {
                displs[i_start] = nodes_passed;
            }
            i_start++;
        }
        nodes_passed++;
    }
    for (int i_thread = 0; i_thread < n_threads - 1; i_thread++) {
        recvcounts[i_thread] = displs[i_thread + 1] - displs[i_thread];
    }
    recvcounts[n_threads - 1] = n_nodes - displs[n_threads - 1];
    int key_estim [2];
    int key_parent [2];
    int key_early_stopping;
    
    int shmid_estim [2];
    int shmid_parent [2];
    int shmid_early_stopping;
    
    for (int par = 0; par < 2; par++) {
        key_estim[par] = par + 1;
        key_parent[par] = par + 3;
        
        shmid_estim[par] = shmget(key_estim[par], sizeof(int)*n_nodes, IPC_CREAT);
        shmid_parent[par] = shmget(key_parent[par], sizeof(int)*n_nodes, IPC_CREAT);
        estim[par] = (int*) shmat(shmid_estim[par], 0, 0);
        parent[par] = (int*) shmat(shmid_parent[par], 0, 0);
    }
    
    key_early_stopping = 5;
    shmid_early_stopping = shmget(key_early_stopping, sizeof(bool), IPC_CREAT);
    bool& early_stopping = *((bool*) shmat(shmid_early_stopping, 0, 0));
    
    MPI_Barrier(MPI_COMM_WORLD);
    
    if (rank == 0) {
        for (Edges::iterator it = edges.begin(); it != edges.end(); it++) {
            int i_node = it->first;
            estim[0][i_node] = INT_MAX / 2;
            estim[1][i_node] = INT_MAX / 2;
            parent[0][i_node] = -1;
            parent[1][i_node] = -1;
        }
        estim[0][src] = 0;
        estim[1][src] = 0;
    }
    
    MPI_Barrier(MPI_COMM_WORLD);
    
    bool parity = false;
    early_stopping = true;
    bool* threads_waiting = new bool [n_threads];
    bool all_done = false;
    
    std::vector<pthread_t*> threads;
    std::vector<thread_arg*> thread_args;
    
    for (int i_pass = 0; i_pass < n_nodes; i_pass++) {
        int from = displs[rank];
        int to;
        if (rank == n_threads - 1) {
            to = n_nodes;
        } else {
            to = displs[rank + 1];
        }
        threads_waiting[rank] = true;
        thread_arg* thr_arg = new thread_arg;
        thread_args.push_back(thr_arg);
        thr_arg->from = from;
        thr_arg->to = to;
        thr_arg->parity = &parity;
        thr_arg->early_stopping = &early_stopping;
        thr_arg->waiting = threads_waiting + rank;
        thr_arg->all_done = &all_done;
        thr_arg->nodes = &nodes;
        thr_arg->edges = &edges;
        thr_arg->parent = parent;
        thr_arg->estim = estim;
        void* arg = (void*) thr_arg;
        bf_thread(arg);
        
        
        MPI_Barrier(MPI_COMM_WORLD);
        
        if (early_stopping && optimize) {
            break;
        }
        
        MPI_Barrier(MPI_COMM_WORLD);
        
        early_stopping = true;
    
        parity = !parity;
    }
    all_done = true;
    
    if (rank == 0) {
        if (estim[!parity][dst] == INFINITY) {
            for (int par = 0; par < 2; par++) {
                shmctl(shmid_estim[par], IPC_RMID, NULL);
                shmctl(shmid_parent[par], IPC_RMID, NULL);
            }
            return std::vector<int>();
        } else {
            std::vector<int> path;
            int elem_cur = dst;
            while (true) {
                path.push_back(elem_cur);
                if (elem_cur == src)
                    break;
                elem_cur = parent[!parity][elem_cur];
            }
            std::reverse(path.begin(), path.end());
            for (int par = 0; par < 2; par++) {
                shmctl(shmid_estim[par], IPC_RMID, NULL);
                shmctl(shmid_parent[par], IPC_RMID, NULL);
            }
            return path;
        }
    } else {
        return std::vector<int>(); 
    }
    
}


int main(int argc, char** argv) {
    MPI_Init(&argc, &argv);
    graph g;
    g.read("graph");
    
    int src, dst;
    src = 0;
    dst = 1;
    
    if (argc > 1) {
        optimize = (bool) atoi(argv[1]);
    } else {
        optimize = false;
    }
    
    int rank;
    int n_threads;
    MPI_Comm_rank(MPI_COMM_WORLD, &rank);
    MPI_Comm_size(MPI_COMM_WORLD, &n_threads);
    
    struct timespec start, stop;
    double elapsed;
    //~ while (true) {
        clock_gettime(CLOCK_MONOTONIC, &start);
        std::vector<int> path = shortest_path_pthread(g, src, dst);
        if (rank == 0) {
            clock_gettime(CLOCK_MONOTONIC, &stop);
            elapsed = (stop.tv_sec - start.tv_sec);
            elapsed += (stop.tv_nsec - start.tv_nsec) / 1000000000.0;
            for (std::vector<int>::iterator it = path.begin(); it != path.end(); it++) {
                printf("%i ", *it);
            }
            printf("\n%lf\n", elapsed);
        //~ }
    }
    
    MPI_Finalize();
	return 0;
}

