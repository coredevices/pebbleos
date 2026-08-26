# SPDX-FileCopyrightText: 2024 Google LLC
# SPDX-License-Identifier: Apache-2.0

"""

Credit to Bernd Klein for this graph class.
http://www.python-course.eu/graphs_python.php

Modifications by kmisquitta:
1) Added support for pretty printing
2) Added function to return a vertex's neighbours
3) Added support for traversing to a vertex more than once in find_all_paths
4) Removed support for adding edges that are sets
5) Removed support for multiple edges between two vertices
6) Added support for traversing beyond the end vertex in find_all_paths
7) Removed many unneeded features

"""

""" A Python Class
A simple Python graph class, demonstrating the essential 
facts and functionalities of graphs.
"""

import pprint


def is_line_segment_in_path(path, vertex_1, vertex_2):
    for i in range(len(path) - 1):
        if (
            path[i] == vertex_1
            and path[i + 1] == vertex_2
            or path[i] == vertex_2
            and path[i + 1] == vertex_1
        ):
            return True
    return False


class Graph:
    def __init__(self, graph_dict={}):
        """initializes a graph object"""
        self.__graph_dict = graph_dict

    def get_vertices(self):
        """returns the vertices of a graph"""
        return list(self.__graph_dict.keys())

    def get_edges(self):
        """returns the edges of a graph"""
        return self.__generate_edges()

    def get_neighbours(self, vertex):
        """returns the neighbours of a vertex"""
        return list(self.__graph_dict[vertex])

    def add_vertex(self, vertex):
        """If the vertex "vertex" is not in
        self.__graph_dict, a key "vertex" with an empty
        list as a value is added to the dictionary.
        Otherwise nothing has to be done.
        """
        if vertex not in self.__graph_dict:
            self.__graph_dict[vertex] = []

    def add_edge(self, edge):
        """assumes that edge is of type tuple or list"""
        if len(edge) < 2:
            return

        vertex1 = edge[0]
        vertex2 = edge[1]
        if vertex1 in self.__graph_dict:
            if vertex2 not in self.get_neighbours(vertex1):
                self.__graph_dict[vertex1].append(vertex2)
        else:
            self.__graph_dict[vertex1] = [vertex2]

    def __generate_edges(self):
        """A static method generating the edges of the
        graph "graph". Edges are represented as sets
        with one (a loop back to the vertex) or two
        vertices
        """
        edges = []
        for vertex in self.__graph_dict:
            for neighbour in self.__graph_dict[vertex]:
                if {vertex, neighbour} not in edges:
                    edges.append({vertex, neighbour})
        return edges

    def __str__(self):
        res = "vertices: "
        for k in self.__graph_dict:
            res += str(k) + " "
        res += "\nedges: "
        for edge in self.__generate_edges():
            res += str(edge) + " "
        return res

    def find_all_paths(self, start_vertex, end_vertex, path=[]):
        """Recursive function that finds all paths from the start vertex to the end vertex.
        Starts from the start vertex and traverses through vertices until the end vertex is reached.
        If there are untraversed edges when the end vertex is reached, will continue traversing
        to check for paths back to the end vertex (loops).
        There is no limit to how many times a vertex can be traversed.
        An edge may be traversed only once.
        """
        graph = self.__graph_dict
        paths = []
        path = path + [start_vertex]
        if start_vertex == end_vertex:
            # Check if additional traversals is possible
            neighbours = self.get_neighbours(end_vertex)
            no_possible_traversals = True
            for neighbour in neighbours:
                if not is_line_segment_in_path(path, end_vertex, neighbour):
                    no_possible_traversals = False
                    break
            if no_possible_traversals:  # Base case
                return [path]
            else:
                paths.append(path)  # Add current path, continue finding
        if start_vertex not in graph:
            return []
        for vertex in graph[start_vertex]:
            if not is_line_segment_in_path(path, vertex, start_vertex):
                extended_paths = self.find_all_paths(vertex, end_vertex, path)
                for p in extended_paths:
                    paths.append(p)
        return paths

    def prettyprint(self):
        pprint.pprint(self.__graph_dict)
