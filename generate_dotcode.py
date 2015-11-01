import sys

import rosgraph

import pydot
import pygraphviz

sys.path.append(
        '/home/fmontoto/rqt_common_plugins/rqt_graph/src/rqt_graph')

import dotcode

class Node(object):
    def __init(self, incoming=None, outgoing=None, edges=None):
        self.incoming = incoming or set()
        self.outgoing = outgoing or set()
        self.edges = edges or set()

    def add_incoming(self, item):
        self.incoming.add(item)

    def add_outgoing(self, item):
        self.outgoing.add(item)

# Idea taken from PydotFactory
class GraphWrapper(object):
    def __init__(self, graphname='graphname', graph_type='digraph', rank='same',
                 simplify=True, rankdir='TB', ranksep=0.2, compound=True):

        self.graph = pydot.Dot(graphname, graph_type=graph_type, rank=rank,
                rankdir=rankdir, simplify=simplify)
        self.graph.set_ranksep(ranksep)
        self.graph.set_compound(compound)

    @staticmethod
    def _customize_item(item, label=None, color=None, url=None, style=None,
                        shape=None):
        if color:
            item.set_color(color)
        if url:
            item.set_URL(url)
        if label:
            item.set_label(label)
        if style:
            item.set_style(style)
        if shape:
            item.set_shape(shape)

    def add_node(self, nodename, nodelabel=None, shape='box', color=None,
                 url=None):
        if not nodename:
            raise ValueError("Empty nodename")
        #TODO we might want to escape some characters. See pydot
        node = pydot.Node(nodename)
        _customize_item(node, label=nodelabel, shape=shape, color=color,
                        url=url)
        self.graph.add_node(node)

    def add_edge(self, nodename1, nodename2, label=None, simplify=True,
                 style=None, penwidth=1, url=None, color=None):
        edge = pydot.Edge(nodename1, nodename2)
        _customize_item(edge, label=label, style=style, url=url)
        edge.obj_dict['attributes']['penwidth'] = str(penwidth)
        if color:
            edge.obj_dict['attributes']['colorR'] = str(color[0])
            edge.obj_dict['attributes']['colorG'] = str(color[1])
            edge.obj_dict['attributes']['colorB'] = str(color[2])
        self.graph.add_edge(edge)

class MyException(Exception):
    pass

class Generator(object):
    def __init__(self):
        self.dotcode_gen = dotcode.RosGraphDotcodeGenerator()
        self.master = rosgraph.Master('/rostopic')

    def _check_master(self):
        try:
            self.master.getPid()
        except socket.error:
            raise MyException("Master not available")

    def _generate_graph(self, nodes_include=None, nodes_exclude=None,
                        topics_include, topics_exclude):
        nn_nodes = {n for n in self._graph.nn_nodes}
        topics = {n for n in self._graph.nt_nodes}

        edges = {e for e in self._graph.nt_all_edges}

        graph = {}
        for e in edges:
            if e.start not in graph:
                graph[e.start] = Node()
            if e.end not in graph:
                graph[e.end] = Node()
            graph[e.start].add_outgoing(e)
            graph[e.end].add_incoming(e)










    def generate(self):
        self._graph = rosgraph.impl.graph.Graph()
        self._graph.set_master_stale(5.0)
        self._graph.set_node_stale(5.0)
        self._graph.update()
        print("Nodes")
        print(self._graph.nn_nodes)
        print("TopicNodes")
        print(self._graph.nt_nodes)
        print("Node to node")
        print(self._graph.nn_edges)
        for n in self._graph.nn_edges:
            #print(n.start)
            #print(n.label)
            #print(n.end)
            #print(n.key)
            print(n)
        print("Node to topic")
        print(self._graph.nt_edges)
        for n in self._graph.nt_edges:
            print(n)
        print("All Node to topic")
        print(self._graph.nt_all_edges)
        for n in self._graph.nt_all_edges:
            print(n)
        print(self.dotcode_gen.generate_dotcode())

def main():
    g = Generator()
    g.generate()
    pass

if __name__ == '__main__':
    main()
