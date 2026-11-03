#pragma once

#include <iostream>

#include <boost/config.hpp>
#include <boost/graph/adjacency_list.hpp>
#include <boost/graph/breadth_first_search.hpp>
#include <boost/graph/dijkstra_shortest_paths.hpp>
#include <boost/graph/graph_traits.hpp>
#include <boost/graph/named_function_params.hpp>
#include <boost/graph/properties.hpp>
#include <boost/graph/visitors.hpp>

#include "crobot/common/assert.hpp"
#include "crobot/math/matrix.hpp"

namespace boost {
enum vertex_properties_t
{
  vertex_properties
};
enum edge_properties_t
{
  edge_properties
};

enum graph_properties_t
{
  custom_graph_properties
};

BOOST_INSTALL_PROPERTY(vertex, properties);
BOOST_INSTALL_PROPERTY(edge, properties);
BOOST_INSTALL_PROPERTY(graph, properties);
} // namespace boost

namespace crobot::math {

template <
    typename VertexProperty,
    typename EdgeProperty,
    typename GraphProperty = boost::no_property>
class Graph {
public:
  using vertex_descriptor = typename boost::adjacency_list_traits<
      boost::listS,
      boost::listS,
      boost::bidirectionalS>::vertex_descriptor;

  using graph_t = typename boost::adjacency_list<
      boost::listS,
      boost::listS,
      boost::bidirectionalS,
      boost::property<
          boost::vertex_index_t,
          int,
          boost::property<
              boost::vertex_predecessor_t,
              vertex_descriptor,
              boost::property<
                  boost::vertex_distance_t,
                  int,
                  boost::
                      property<boost::vertex_properties_t, VertexProperty>>>>,
      boost::property<
          boost::edge_weight_t,
          int,
          boost::property<boost::edge_properties_t, EdgeProperty>>,
      boost::property<boost::graph_properties_t, GraphProperty>>;

  using Vertex = typename boost::graph_traits<graph_t>::vertex_descriptor;
  using VertexIterator = typename boost::graph_traits<graph_t>::vertex_iterator;
  using Edge           = typename boost::graph_traits<graph_t>::edge_descriptor;
  using EdgeIterator   = typename boost::graph_traits<graph_t>::edge_iterator;
  using EdgePair       = std::pair<Edge, Edge>;

  using VertexRange = std::pair<VertexIterator, VertexIterator>;
  using EdgeRange   = std::pair<EdgeIterator, EdgeIterator>;

  using vertex_index_map_t =
      typename boost::property_map<graph_t, boost::vertex_index_t>::type;
  using vertex_predcessor_map_t =
      typename boost::property_map<graph_t, boost::vertex_predecessor_t>::type;
  using vertex_distance_map_t =
      typename boost::property_map<graph_t, boost::vertex_distance_t>::type;
  using edge_weight_map_t =
      typename boost::property_map<graph_t, boost::edge_weight_t>::type;

  Graph()
    : m_graph(graph_t()),
      m_indexmap(boost::get(boost::vertex_index, m_graph)),
      m_predmap(boost::get(boost::vertex_predecessor, m_graph)),
      m_distmap(boost::get(boost::vertex_distance, m_graph)),
      m_weightmap(boost::get(boost::edge_weight, m_graph)),
      m_vert_index(0) {
  }

  /**
   * @brief Add a vertex to the graph.
   *
   * @param prop Properties of the vertex.
   * @return Vertex descriptor of the added vertex.
   */
  Vertex add_vertex(const VertexProperty& prop) {
    Vertex v             = boost::add_vertex(m_graph);
    vertex_properties(v) = prop;

    m_indexmap[v] = m_vert_index++;

    return v;
  }

  /**
   * @brief Add a pair of edges between two given vertices to the graph.
   * Multiple edges can be added to the same pair of vertices.
   *
   * @param v1 First vertex.
   * @param v2 Second vertex.
   * @param prop_12 Properties of the edge from v1 to v2.
   * @param prop_21 Properties of the edge from v2 to v1.
   * @return EdgePair of the added edges.
   */
  EdgePair add_edge_bidirectional(
      const Vertex& v1,
      const Vertex& v2,
      const EdgeProperty& prop_12,
      const EdgeProperty& prop_21) {
    return EdgePair(add_edge(v1, v2, prop_12), add_edge(v2, v1, prop_21));
  }

  /**
   * @brief Add a edge from v1 to v2 to the graph.
   *
   * @param v1 First vertex.
   * @param v2 Second vertex.
   * @return Edge descriptor of the added edge.
   */
  Edge add_edge(
      const Vertex& v1,
      const Vertex& v2,
      const EdgeProperty& prop = EdgeProperty()) {
    auto edge_info = boost::add_edge(v1, v2, m_graph);
    CROBOT_ASSERT_ERROR(edge_info.second, "Failed to add edge.");
    Edge edge             = edge_info.first;
    edge_properties(edge) = prop;

    using edge_property_map
        = boost::property_map<graph_t, boost::edge_weight_t>;
    typename edge_property_map::type param = get(boost::edge_weight, m_graph);

    param[edge] = 1;
    return edge;
  }

  /**
   * @brief Get the start and end iterators of the vertices.
   *
   * @return VertexRange of the vertices.
   */
  VertexRange get_vertices() const {
    return vertices(m_graph);
  }

  /**
   * @brief Get the number of vertices in the graph.
   *
   * @return int Number of vertices.
   */
  int num_vertices() const {
    return boost::num_vertices(m_graph);
  }

  /**
   * @brief Get the num edges object
   *
   * @return int
   */
  int num_edges() const {
    return boost::num_edges(m_graph);
  }

  /**
   * @brief Get the propertiy of a given vertex.
   *
   * @param v Vertex descriptor.
   * @return VertexProperty& Property of the vertex.
   */
  VertexProperty& vertex_properties(const Vertex& v) {
    using vertex_property_map
        = boost::property_map<graph_t, boost::vertex_properties_t>;
    typename vertex_property_map::type param
        = get(boost::vertex_properties, m_graph);
    return param[v];
  }

  /**
   * @brief Get the propertiy of a given vertex.
   *
   * @param v Vertex descriptor.
   * @return VertexProperty& Property of the vertex.
   */
  const VertexProperty& vertex_properties(const Vertex& v) const {
    using vertex_property_map
        = boost::property_map<graph_t, boost::vertex_properties_t>;
    typename vertex_property_map::const_type param
        = get(boost::vertex_properties, m_graph);
    return param[v];
  }

  /**
   * @brief Get the propertiy of a given edge.
   *
   * @param e Edge descriptor.
   * @return EdgeProperty& Property of the edge.
   */
  EdgeProperty& edge_properties(const Edge& e) {
    using edge_property_map
        = boost::property_map<graph_t, boost::edge_properties_t>;
    typename edge_property_map::type param
        = get(boost::edge_properties, m_graph);
    return param[e];
  }

  /**
   * @brief Get the propertiy of a given edge.
   *
   * @param e Edge descriptor.
   * @return const EdgeProperty& Property of the edge.
   */
  const EdgeProperty& edge_properties(const Edge& e) const {
    using edge_property_map
        = boost::property_map<graph_t, boost::edge_properties_t>;
    typename edge_property_map::const_type param
        = get(boost::edge_properties, m_graph);
    return param[e];
  }

  /**
   * @brief Get the propertiy of the graph.
   *
   * @return GraphProperty& Property of the graph.
   */
  GraphProperty& graph_properties() {
    return boost::get_property(m_graph, boost::custom_graph_properties);
  }

  /**
   * @brief Get the propertiy of the graph.
   *
   * @return const GraphProperty& Property of the graph.
   */
  const GraphProperty& graph_properties() const {
    return boost::get_property(m_graph, boost::custom_graph_properties);
  }

  /**
   * @brief Get the number of outgoing edges of a given vertex.
   *
   * @param v Vertex descriptor.
   * @return int Number of outgoing edges.
   */
  int vertex_outdegree(const Vertex& v) const {
    return boost::out_degree(v, m_graph);
  }

  /**
   * @brief Get the number of incoming edges of a given vertex.
   *
   * @param v Vertex descriptor.
   * @return int Number of incoming edges.
   */
  int vertex_indegree(const Vertex& v) const {
    return boost::in_degree(v, m_graph);
  }

  auto get_vertex_inedges(const Vertex& v) const {
    return boost::in_edges(v, m_graph);
  }

  Vertex edge_source(const Edge& e) const {
    return boost::source(e, m_graph);
  }

  /**
   * @brief Get the shortest path from a given vertex to a given target.
   *
   * @param start Start vertex.
   * @param end End vertex.
   * @return std::vector<Vertex> Path from start to end.
   */
  std::vector<Vertex> get_shortest_path(
      const Vertex& start, const Vertex& end) {
    using namespace boost;
    std::vector<Vertex> predecessors(boost::num_vertices(m_graph));
    dijkstra_shortest_paths(
        m_graph, start, predecessor_map(m_predmap).distance_map(m_distmap));

    std::vector<Vertex> path;
    for (Vertex v = end; v != start; v = m_predmap[v]) {
      path.push_back(v);
    }
    path.push_back(start);
    std::reverse(path.begin(), path.end());
    return path;
  }

  /**
   * @brief Backtrack the shortest path from a given vertex.
   *
   * @param end End vertex.
   * @return std::vector<Vertex> Path from start to end.
   */
  std::vector<Vertex> backtrack_path(const Vertex& end) {
    std::vector<Vertex> path;
    Vertex vi = end;
    while (boost::in_degree(vi, m_graph) > 0) {
      path.push_back(vi);
      vi = boost::source(*boost::in_edges(vi, m_graph).first, m_graph);
    }
    path.push_back(vi);
    std::reverse(path.begin(), path.end());
    return path;
  }

  /**
   * @brief Remove all vertices from the graph.
   */
  void remove_all_vertices() {
    VertexIterator vi, vi_end, next;
    std::tie(vi, vi_end) = boost::vertices(m_graph);
    for (next = vi; vi != vi_end; vi = next) {
      ++next;
      boost::remove_vertex(*vi, m_graph);
    }
  }

  /**
   * @brief Clear the graph.
   */
  void clear() {
    m_graph.clear();
  }

private:
  graph_t m_graph;

  vertex_index_map_t m_indexmap;
  vertex_predcessor_map_t m_predmap;
  vertex_distance_map_t m_distmap;
  edge_weight_map_t m_weightmap;

  int m_vert_index;
};

} // namespace crobot::math
