import os
import pydot
import socket
import sys
import time

import rosgraph
from rosgraph.impl import graph

import rwlock


class UnreachableRos(Exception):
    """Raised when ROS not reachable."""
    pass


def escape_text(text):
    return text
    # return text.replace('/', '_').replace('%', '_').replace('-', '_')


class Node(object):
    def __init__(self, name, incoming=None, outgoing=None, edges=None):
        self.name = name
        self.incoming = incoming or set()
        self.outgoing = set()
        self.edges = edges or set()

    def add_incoming(self, item):
        self.incoming.add(item)

    def add_outgoing(self, item):
        self.outgoing.add(item)


# Idea taken from PydotFactory
class GraphWrapper(object):
    def __init__(self, graphname='graphname', graph_type='digraph', rank='same',
                 simplify=True, rankdir='LR', ranksep=0.2, compound=True,
                 graph=None):
        self.subgraphs = dict()
        if graph:
            self.graph = graph
            return

        self.graph = pydot.Dot(escape_text(graphname), graph_type=graph_type,
                               rank=rank, rankdir=rankdir, simplify=simplify,
                               compound=compound, ratio='fill')
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

    def add_node(self, nodename, nodelabel=None, shape='circle', color=None,
                 url=None):
        if not nodename:
            raise ValueError("Empty nodename")
        # TODO we might want to escape some characters. See pydot
        node = pydot.Node(escape_text(nodename))
        self._customize_item(node, label=nodelabel, shape=shape, color=color,
                             url=url)
        self.graph.add_node(node)

    def add_edge(self, nodename1, nodename2, label=None, simplify=True,
                 style=None, penwidth=1, url=None, color=None):
        if label:
            label = escape_text(label)
        edge = pydot.Edge(nodename1, nodename2)
        self._customize_item(edge, label=None, style=style, url=url)
        edge.obj_dict['attributes']['penwidth'] = str(penwidth)
        if color:
            edge.obj_dict['attributes']['colorR'] = str(color[0])
            edge.obj_dict['attributes']['colorG'] = str(color[1])
            edge.obj_dict['attributes']['colorB'] = str(color[2])
        self.graph.add_edge(edge)

    def add_cluster(self, name, label=None, rank='same', rankdir='TB',
                    simplify=True, color=None, shape='circle', compound=True,
                    style='bold', ranksep=0.2, orientation='portrait'):

        name = escape_text(name)
        c = pydot.Cluster(escape_text(name), rank=rank, rankdir=rankdir,
                          simplify=simplify)
        if 'set_shape' in c.__dict__:
            c.set_shape(shape)
        if 'set_style' in c.__dict__:
            c.set_style(style)
        if 'set_color' in c.__dict__:
            c.set_color(color)
        c.set_compound(compound)
        c.set_ranksep(ranksep)
        c.set_label(label or name)
        self.graph.add_subgraph(c)
        self.subgraphs[name] = GraphWrapper(graph=c)
        # print(c.get_name())
        return self.subgraphs[name]

    def get_cluster(self, name):
        return self.subgraphs[escape_text(name)]

    def generate_dot(self, path):
        self.graph.write_svg(path)
        return self.graph


class Generator(object):
    def __init__(self):
        self.last_svg_generated = None
        self.last_svg_generated_ts = 0
        self.rwlock = rwlock.RWLock()
        self.master = rosgraph.Master('/rostopic')

    def _check_master(self):
        try:
            self.master.getPid()
        except socket.error:
            raise UnreachableRos("Master not available")

    @staticmethod
    def _filter_nodes(nodes, exclude):
        if exclude is None:
            return {n.strip() for n in nodes}
        ret = set()
        for n_ in nodes:
            n = n_.strip()
            skip = False
            for e in exclude:
                if n.startswith(e):
                    skip = True
                    break
            if not skip:
                ret.add(n)
        return ret

    def _generate_graph(self, path, nodes_include=None, nodes_exclude=None,
                        topics_include=None, topics_exclude=None,
                        hide_dead_sinks=True):

        nn_nodes = self._filter_nodes(self._graph.nn_nodes, nodes_exclude)
        topics = self._filter_nodes(self._graph.nt_nodes, topics_exclude)

        edges = {e for e in self._graph.nt_all_edges}

        graph_dict = {}
        for e in edges:
            start = e.start.strip()
            end = e.end.strip()
            if start not in graph_dict:
                graph_dict[start] = Node(start)
            if end not in graph_dict:
                graph_dict[end] = Node(end)
            graph_dict[start].add_outgoing(e)
            graph_dict[end].add_incoming(e)

        graph = GraphWrapper()
        for node in nn_nodes:
            graph.add_node(node, shape='box')

        if hide_dead_sinks:
            filtered_topics = [e for e in topics
                               if e in graph_dict and
                               len(graph_dict[e].outgoing) > 0]
        else:
            filtered_topics = topics

        # Clustering
        last_namespace = ""
        namespaces = set()
        str_edges = [e.split('/')[1:] for e in sorted(filtered_topics)]
        for spl in str_edges:
            if spl[0] == last_namespace:
                namespaces.add(last_namespace)
            last_namespace = spl[0]

        for n in namespaces:
            graph.add_cluster(n)

        for topic in filtered_topics:
            try:
                nspace = topic.strip()[1:topic.index('/', 1)]
            except ValueError:
                nspace = topic.strip()[1:]
            if nspace in namespaces:
                graph.get_cluster(nspace).add_node(topic, shape='diamond')
            else:
                graph.add_node(topic, shape='diamond')

        for e in edges:
            start = e.start.strip()
            end = e.end.strip()
            exclude = False
            # TODO check this
            if (start not in filtered_topics and
                        end not in filtered_topics):
                continue
            if nodes_exclude is not None:
                for n in nodes_exclude:
                    if end.startswith(n) or start.startswith(n):
                        exclude = True
                        break
                if exclude:
                    continue
            graph.add_edge(start, end, label=str(e))
        dot = graph.generate_dot(path)

    def generate(self, path, **kwargs):
        self._graph = rosgraph.impl.graph.Graph()
        self._graph.set_master_stale(5.0)
        self._graph.set_node_stale(5.0)
        try:
            self._graph.update()
        except socket.error:
            raise UnreachableRos("Ros not reachable")
        self._generate_graph(path, **kwargs)

    def get_current_svg(self, prefix="", file_name=None,
                        max_time_to_refresh=5, **kwargs):
        # TODO when do we remove old files ?
        with rwlock.read_lock(self.rwlock):
            now = time.time()
            name = file_name if file_name else "%s.svg" % now
            path = os.path.join(prefix, name)

            if now - self.last_svg_generated_ts > max_time_to_refresh:
                self.rwlock.promote()
                self.generate(path, **kwargs)
                self.last_svg_generated = path
                self.last_svg_generated_ts = now

            return self.last_svg_generated


def main():
    g = Generator()
    g.generate()
    pass


if __name__ == '__main__':
    main()
