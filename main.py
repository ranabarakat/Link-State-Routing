import networkx as nx
import matplotlib.pyplot as plt
import sys


def init(graph_data):
    graph = nx.Graph()
    # for _ in range(max(num_nodes, num_edges)):
    for data in graph_data:
        # n1, n2, w = input().split(',')
        # graph.add_node(str(n1))
        # graph.add_node(str(n2))
        # graph.add_edge(str(n1), str(n2), weight=float(w))
        graph.add_node(str(data[0]))
        graph.add_node(str(data[1]))
        graph.add_edge(str(data[0]), str(data[1]), weight=float(data[2]))
    return graph


def visualize(G):
    plt.figure(figsize=(20, 20))
    # positions for all nodes - seed for reproducibility
    pos = nx.spring_layout(G, seed=7)

    # nodes
    nx.draw_networkx_nodes(G, pos, node_size=700)

    # edges
    nx.draw_networkx_edges(G, pos)

    # node labels
    nx.draw_networkx_labels(G, pos, font_size=20, font_family="sans-serif")

    # edge weight labels
    edge_labels = nx.get_edge_attributes(G, "weight")
    nx.draw_networkx_edge_labels(G, pos, edge_labels, font_size=8)
    plt.show()


def dijkstra(G, src):
    visited = {}
    dist = {}
    predecessors = {}
    for node in G.nodes:
        visited[node] = False
        dist[node] = float('inf')
        predecessors[node] = None

    dist[src] = 0

    while True:
        min_dist = float('inf')
        for node in G.nodes:
            if dist[node] < min_dist and not visited[node]:
                visited[node] = True
                min_dist = dist[node]
                curr_node = node

        if min_dist == float('inf'):
            break
        for neighbor in G.neighbors(curr_node):
            # dist[neighbor] = min(dist[neighbor],min_dist + G.get_edge_data(curr_node,neighbor)['weight'])
            temp = min_dist + G.get_edge_data(curr_node, neighbor)['weight']
            if temp < dist[neighbor]:
                predecessors[neighbor] = curr_node
                dist[neighbor] = temp

    return dist, predecessors


def gen_forwarding_table(G, src, dst):
    _, predecessors = dijkstra(G, src)
    path = []
    # original_dst = dst
    while not dst == None:
        path.append(dst)
        dst = predecessors[dst]
    path.reverse()
    # print(f"path from {src} to {original_dst}: {path}")
    return src, path[1]

    # node = nx.dijkstra_path(G, src, dst,weight="weight")[1]
    # return src,node


def print_tables(G):
    for node in G.nodes:
        print(f"\n-------- Forwarding table of node {node} --------\n")
        print(f"Destination\tLink")
        for dst in G.nodes:
            if not node == dst:
                print(f"{dst}\t\t", end="")
                src, nxt = gen_forwarding_table(G, node, dst)
                print(f"({src},{nxt})")


if __name__ == '__main__':
    # nodes, edges = map(int, input().split(','))
    graph_data = []
    with open('input.txt') as f:
        nodes, edges = map(int, f.readline().split(','))
        for line in f:
            n1, n2, weight = line.split(',')
            graph_data.append([n1, n2, weight])

    # G = init(nodes, edges)
    print(graph_data)
    G = init(graph_data)
    # dist,pred = dijkstra(G,'u')
    # print(dist)
    # print(pred)
    print_tables(G)
    visualize(G)


# u,v,2
# u,w,5
# u,x,1
# x,v,2
# v,w,3
# x,w,3
# w,y,1
# x,y,1
# w,z,5
# y,z,2
