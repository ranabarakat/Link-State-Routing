import networkx as nx
import matplotlib as plt
import sys


def init(num_nodes, num_edges):
    graph = nx.Graph()
    for _ in range(max(num_nodes, num_edges)):
        n1, n2, w = input().split(',')
        graph.add_node(str(n1))
        graph.add_node(str(n2))
        graph.add_edge(str(n1), str(n2), weight=float(w))
    return graph


def visualize(G):
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
    nx.draw_networkx_edge_labels(G, pos, edge_labels)

    # ax = plt.gca()
    # ax.margins(0.08)
    plt.axis("off")
    plt.tight_layout()
    plt.show()

def dijkstra(G,src):
    visited = {}
    dist = {}
    for node in G.nodes:
        visited[node] = False
        dist[node] = float('inf')

    dist[src] = 0

    while True:
        min_dist = float('inf')
        for node in G.nodes:
            if dist[node]<min_dist and not visited[node]:
                min_dist = dist[node]
                curr_node = node
        if min_dist == float('inf'): 
            break
        for neighbor in G.neighbors(curr_node):
            dist[neighbor] = min(dist[neighbor],min_dist + G.get_edge_data(curr_node,neighbor)['weight'])

    return dist


if __name__ == '__main__':
    # nodes,edges = sys.argv[1:3]
    nodes, edges = map(int, input().split(','))

    G = init(nodes, edges)
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
