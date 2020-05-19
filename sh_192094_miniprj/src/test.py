from sklearn.neighbors import KDTree
import numpy as np
import networkx as nx

G = nx.Graph()  # A graph to hold the nearest neighbours

X = [(1.5, 0), (2.5, 2.5), (-3.5, 2), (1.5, 4.5), (-3.25, -2.75), (3, -7), (-0.5, 6.5 ), (0, 2.5), (-3.25, -0.25 ), (2, 12), (-3.25, 7 ), (-3, 3), (3, -1), (0, -1.5), (-4.5, -1.75), (-11, 2.5), (-3, -4), (-3.25, 10), (-2.75, -9), (-1, -0.5)]  # Some list of points in 2D
#tree = KDTree(X, leaf_size=2, metric='minkowski')  # Create a distance tree
tree = KDTree(X, leaf_size=40)#, metric='minkowski')

# Now loop over your points and find the two nearest neighbours
# If the first and last points are also the start and end points of the line you can use X[1:-1]
for p in X:
    dist, ind = tree.query(p, k=2)
    print (ind)

    # ind Indexes represent nodes on a graph
    # Two nearest points are at indexes 1 and 2. 
    # Use these to form edges on graph
    # p is the current point in the list
    G.add_node(p)
    n1, l1 = X[ind[0][1]], dist[0][1]  # The next nearest point
    n2, l2 = X[ind[0][2]], dist[0][2]  # The following nearest point  
    G.add_edge(p, n1)
    G.add_edge(p, n2)


print (G.edges())  # A list of all the connections between points
print (nx.shortest_path(G, source=(1.5,0), target=(-1,-0.5), method='euclidean'))
