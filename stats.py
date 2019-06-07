import networkx as nx
import random
import matplotlib.pyplot as plt
import numpy as np
import subprocess
import pandas as pd
import math

from time import sleep

# pwd = '****'
def run(method, n_threads=1, optimize=False):
    proc_pwd = subprocess.Popen(['echo', pwd], stdout=subprocess.PIPE)
    proc = subprocess.Popen(['sudo -S ' + calls(method, n_threads)], stdin=proc_pwd.stdout, stdout=subprocess.PIPE, shell=True)

    path = proc.stdout.readline().decode('utf-8').strip().split()
    path = [int(i) for i in path]
    time = float(proc.stdout.readline())
    return path, time

def calls(case, n_threads=1, optimize=False):
    calls = {
        'serial' : lambda n, o: './serial {}'.format(o),
        'pthread': lambda n, o: './pthread {} {}'.format(n, o),
        'mpi'    : lambda n, o: 'mpirun -n {} ./mpi {}'.format(n, o),
    }
    return calls[case](n_threads, optimize)

n_tests = 100
def path_len(graph, path):
    return sum([graph[path[i]][path[i + 1]]['weight'] for i in range(len(path) - 1)])


# тестирование корректности реализации
for i_test in range(n_tests):
    n_nodes = random.randint(10, 100)
    n_edges = random.randint(n_nodes, n_nodes ** 2)
    g = (nx.gnm_random_graph(n_nodes, n_edges, directed=True))
    for (u,v,w) in g.edges(data=True):
        w['weight'] = random.randint(0,10)
    nx.write_weighted_edgelist(g, 'graph')
    
    path, time = run('serial')


    true_len = path_len(g, nx.shortest_path(g, 0, 1, weight='weight'))
    comp_len = path_len(g, path)
    if nx.has_path(g, 0, 1) and true_len < comp_len:
        print("Error")
        

# зависимость времени выполнения от числа потоков (процессов)
methods = ['serial', 'pthread', 'mpi']
times = {};
n_runs = 4
max_threads = 4
for method in methods:
    for n_threads in range(1, max_threads + 1):
        times[(method, n_threads)] = []
        n_threads_ = min(n_threads, 4) if method == 'mpi' else n_threads
        for i_run in range(n_runs):
            print((method, n_threads))
            path, time = run(method, n_threads_)
            times[(method, n_threads)].append(time)
#             sleep(0.2)
df = pd.DataFrame(times)
df.describe()


# зависимость ускорения от размера задачи
ratios = {};
n_runs = 4
n_rows = 8
ns_nodes = np.logspace(1, math.log(6000, 10), n_rows, dtype=int)

for n_nodes in ns_nodes:
    while True:
        g = (nx.gnm_random_graph(n_nodes, n_nodes*2, directed=True))
        if not nx.has_path(g, 0, 1):
            continue
        break
    for (u,v,w) in g.edges(data=True):
        w['weight'] = random.randint(0,10)
    nx.write_weighted_edgelist(g, 'graph')

    ratios[n_nodes] = []
    for i_run in range(n_runs):
        print((n_nodes, n_edges))
        path, time1 = run('serial')
        path, time4 = run('pthread', 4)
        ratios[n_nodes].append(time1 / time4)
#             sleep(0.2)
    ratios[n_nodes] = np.mean(ratios[n_nodes])
