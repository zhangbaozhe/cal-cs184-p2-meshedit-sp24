#include "student_code.h"
#include "mutablePriorityQueue.h"

#include <cassert>
#include <chrono>
#define START_TICK auto t1 = chrono::high_resolution_clock::now()
#define END_TICK do { \
    auto t2 = chrono::high_resolution_clock::now(); \
    chrono::duration<double, std::milli> ms_double = t2 - t1; \
    cout << __FUNCTION__ << " using " << ms_double.count() << " ms" << endl; \
  } while(0)

using namespace std;

namespace CGL
{

  /**
   * Evaluates one step of the de Casteljau's algorithm using the given points and
   * the scalar parameter t (class member).
   *
   * @param points A vector of points in 2D
   * @return A vector containing intermediate points or the final interpolated vector
   */
  std::vector<Vector2D> BezierCurve::evaluateStep(std::vector<Vector2D> const &points)
  { 
    // TODO Part 1.
    std::vector<Vector2D> results;

    // Perform de Casteljau's algorithm
    for (size_t i = 0; i < points.size() - 1; i++) {
      Vector2D interpolated_point = (1 - t) * points[i] + t * points[i + 1];
      results.emplace_back(std::move(interpolated_point));
    }

    return results;
  }

  /**
   * Evaluates one step of the de Casteljau's algorithm using the given points and
   * the scalar parameter t (function parameter).
   *
   * @param points    A vector of points in 3D
   * @param t         Scalar interpolation parameter
   * @return A vector containing intermediate points or the final interpolated vector
   */
  std::vector<Vector3D> BezierPatch::evaluateStep(std::vector<Vector3D> const &points, double t) const
  {
    // TODO Part 2.

    std::vector<Vector3D> results;

    for (size_t i = 0; i < points.size() - 1; i++) {
      Vector3D interpolated_point = (1 - t) * points[i] + t * points[i + 1];
      results.emplace_back(std::move(interpolated_point));
    }
    return results;
  }

  /**
   * @brief Compile-time binomial coefficient
   * 
   * @param n 
   * @param k 
   * @return constexpr size_t 
   */
  constexpr size_t binomial_coefficient(size_t n, size_t k) noexcept {
    return 
      (k > n) ? 0 : 
      (k == 0 || k == n) ? 1 :
      (k == 1 || k == n - 1) ? n : 
      (k + k < n) ? 
        (binomial_coefficient(n-1, k-1) * n) / k : 
        (binomial_coefficient(n-1, k) * n) / (n-k);
  }

  /**
   * Fully evaluates de Casteljau's algorithm for a vector of points at scalar parameter t
   *
   * @param points    A vector of points in 3D
   * @param t         Scalar interpolation parameter
   * @return Final interpolated vector
   */
  Vector3D BezierPatch::evaluate1D(std::vector<Vector3D> const &points, double t) const
  {
    // TODO Part 2.


    Vector3D result;
#ifdef IS_ALGEBRAIC
    const size_t n = points.size() - 1;

    // directly using the algebraic form
    for (size_t i = 0; i < points.size(); i++) {
      
      result += (double)binomial_coefficient(n, i) * pow(1-t, n-i) * pow(t, i) * points[i];
    }
#else
    std::vector<Vector3D> temp = points;
    for (size_t i = 0; i < points.size() - 1; i++) {
      temp = evaluateStep(temp, t);
    }    
    result = temp[0];

#endif
    return result;
  }

  // helper function for getting the area of a face
  double area(const FaceIter &f) {
    HalfedgeCIter h = f->halfedge();
    Vector3D a = h->vertex()->position;
    Vector3D b = h->next()->vertex()->position;
    Vector3D c = h->next()->next()->vertex()->position;
    return cross(b - a, c - a).norm() / 2;
  }

  double area(FaceCIter f) {
    HalfedgeCIter h = f->halfedge();
    Vector3D a = h->vertex()->position;
    Vector3D b = h->next()->vertex()->position;
    Vector3D c = h->next()->next()->vertex()->position;
    return cross(b - a, c - a).norm() / 2;
  }

  /**
   * Evaluates the Bezier patch at parameter (u, v)
   *
   * @param u         Scalar interpolation parameter
   * @param v         Scalar interpolation parameter (along the other axis)
   * @return Final interpolated vector
   */
  Vector3D BezierPatch::evaluate(double u, double v) const 
  {  
    // TODO Part 2.
    // START_TICK;
    std::vector<Vector3D> sub_control_points; 
    for (const auto & row : controlPoints) {
      sub_control_points.emplace_back(std::move(
          evaluate1D(row, u)
      ));
    }
    auto result = evaluate1D(sub_control_points, v);
    // END_TICK;
    return result;
  }

  Vector3D Vertex::normal( void ) const
  {
    // TODO Part 3.
    // Returns an approximate unit normal at this vertex, computed by
    // taking the area-weighted average of the normals of neighboring
    // triangles, then normalizing.

    Vector3D normal(0, 0, 0);

    // Get the halfedge of the vertex
    HalfedgeCIter h = this->halfedge();

    do {
      
      FaceCIter f = h->face();
      if (f->isBoundary()) {
        h = h->twin()->next();
        continue;
      }
      double area = ::area(f);
      normal += area * f->normal();

      h = h->twin()->next();
    } while(h != this->halfedge());

    return normal.unit();
  }

  EdgeIter HalfedgeMesh::flipEdge( EdgeIter e0 )
  {
    // TODO Part 4.
    // This method should flip the given edge and return an iterator to the flipped edge.

    if (e0->isBoundary())
      return e0;

    auto h0 = e0->halfedge();
    auto h1 = h0->next();
    auto h2 = h1->next();
    auto h3 = h0->twin();
    auto h4 = h3->next();
    auto h5 = h4->next();

    auto h6 = h1->twin();
    auto h7 = h2->twin();
    auto h8 = h4->twin();
    auto h9 = h5->twin();

    auto v0 = h0->vertex();
    auto v1 = h3->vertex();
    auto v2 = h2->vertex();
    auto v3 = h5->vertex();

    auto e1 = h1->edge();
    auto e2 = h2->edge();
    auto e3 = h4->edge();
    auto e4 = h5->edge();

    auto f0 = h0->face();
    auto f1 = h3->face();

    // changing
    h0->setNeighbors(h1, h3, v3, e0, f0);
    h1->setNeighbors(h2, h7, v2, e2, f0);
    h2->setNeighbors(h0, h8, v0, e3, f0);

    h3->setNeighbors(h4, h0, v2, e0, f1);
    h4->setNeighbors(h5, h9, v3, e4, f1);
    h5->setNeighbors(h3, h6, v1, e1, f1);

    h6->setNeighbors(h6->next(), h5, v2, e1, h6->face());
    h7->setNeighbors(h7->next(), h1, v0, e2, h7->face());
    h8->setNeighbors(h8->next(), h2, v3, e3, h8->face());
    h9->setNeighbors(h9->next(), h4, v1, e4, h9->face());

    v0->halfedge() = h2;
    v1->halfedge() = h5;
    v2->halfedge() = h1;
    v3->halfedge() = h4;

    e0->halfedge() = h0;
    e1->halfedge() = h5;
    e2->halfedge() = h1;
    e3->halfedge() = h2;
    e4->halfedge() = h4;
    
    f0->halfedge() = h0;
    f1->halfedge() = h3;


    return e0;

  }

  /**
   * @brief Helper function with an 
   * extra parameter to reserve the new added edge for later to set its `isNew` 
   * to false, because it is split from an old edge.
   * 
   * @param mesh 
   * @param e0 
   * @param e_reserve 
   * @param is_set_new
   * @return VertexIter 
   */
  VertexIter splitEdgeWithReserve(HalfedgeMesh& mesh, EdgeIter e0, EdgeIter &e_reserve, bool is_set_new = true) {
    if (e0->isBoundary()) {
      auto h0 = e0->halfedge();
      auto h1 = h0->next();
      auto h2 = h1->next();
      auto h3 = h0->twin(); // virtual boundary halfedge
      auto h4 = h1->twin();
      auto h5 = h2->twin();

      auto v0 = h0->vertex();
      auto v1 = h1->vertex();
      auto v2 = h2->vertex();

      auto e1 = h1->edge();
      auto e2 = h2->edge();

      auto f0 = h0->face();
      auto f1 = h3->face();
      // assert(f1->isBoundary());

      // allocating
      auto v_new0 = mesh.newVertex();

      auto e_new0 = mesh.newEdge();
      auto e_new1 = mesh.newEdge();

      auto f_new0 = mesh.newFace();

      auto h_new0 = mesh.newHalfedge();
      auto h_new1 = mesh.newHalfedge();
      auto h_new2 = mesh.newHalfedge();
      auto h_new3 = mesh.newHalfedge();

      // modifying
      v_new0->position = (v0->position + v1->position) / 2;
      v_new0->halfedge() = h_new2;
      v_new0->isNew = is_set_new;
      v0->halfedge() = h0;
      v1->halfedge() = h1;
      v2->halfedge() = h2;

      e0->halfedge() = h0;
      e_new0->halfedge() = h_new0;
      e_new0->isNew = is_set_new;
      e_new1->halfedge() = h_new2;
      e_new1->isNew = is_set_new; 

      // e_reserve for later use
      e_reserve = e_new1;

      f0->halfedge() = h0;
      f_new0->halfedge() = h_new2;

      h0->setNeighbors(h_new0, h3, v0, e0, f0);
      h_new0->setNeighbors(h2, h_new1, v_new0, e_new0, f0);
      h2->setNeighbors(h0, h5, v2, e2, f0);

      h_new1->setNeighbors(h_new2, h_new0, v2, e_new0, f_new0);
      h_new2->setNeighbors(h1, h_new3, v_new0, e_new1, f_new0);
      h1->setNeighbors(h_new1, h4, v1, e1, f_new0);

      h3->setNeighbors(h3->next(), h0, v_new0, e0, f1);
      h_new3->setNeighbors(h3, h_new2, v1, e_new1, f1);

      return v_new0;
    }

    auto h0 = e0->halfedge();
    auto h1 = h0->next();
    auto h2 = h1->next();
    auto h3 = h0->twin();
    auto h4 = h3->next();
    auto h5 = h4->next();

    auto h6 = h1->twin();
    auto h7 = h2->twin();
    auto h8 = h4->twin();
    auto h9 = h5->twin();

    auto v0 = h0->vertex();
    auto v1 = h3->vertex();
    auto v2 = h2->vertex();
    auto v3 = h5->vertex();

    auto e1 = h1->edge();
    auto e2 = h2->edge();
    auto e3 = h4->edge();
    auto e4 = h5->edge();

    auto f0 = h0->face();
    auto f1 = h3->face();

    // allocating
    auto v_new0 = mesh.newVertex();
    auto e_new0 = mesh.newEdge();
    auto e_new1 = mesh.newEdge();
    auto e_new2 = mesh.newEdge();
    auto f_new0 = mesh.newFace();
    auto f_new1 = mesh.newFace();
    auto h_new0 = mesh.newHalfedge();
    auto h_new1 = mesh.newHalfedge();
    auto h_new2 = mesh.newHalfedge();
    auto h_new3 = mesh.newHalfedge();
    auto h_new4 = mesh.newHalfedge();
    auto h_new5 = mesh.newHalfedge();

    // modifying
    h1->setNeighbors(h_new0, h6, v1, e1, f0);
    h_new0->setNeighbors(h0, h_new1, v2, e_new1, f0);
    h0->setNeighbors(h1, h3, v_new0, e0, f0);

    h3->setNeighbors(h_new5, h0, v1, e0, f1);
    h_new5->setNeighbors(h5, h_new4, v_new0, e_new2, f1);
    h5->setNeighbors(h3, h9, v3, e4, f1);

    h_new1->setNeighbors(h2, h_new0, v_new0, e_new1, f_new0);
    h2->setNeighbors(h_new2, h7, v2, e2, f_new0);
    h_new2->setNeighbors(h_new1, h_new3, v0, e_new0, f_new0);

    h_new3->setNeighbors(h4, h_new2, v_new0, e_new0, f_new1);
    h4->setNeighbors(h_new4, h8, v0, e3, f_new1);
    h_new4->setNeighbors(h_new3, h_new5, v3, e_new2, f_new1);

    v_new0->position = (v0->position + v1->position) / 2;
    v_new0->halfedge() = h_new1;
    v_new0->isNew = is_set_new;
    v1->halfedge() = h3;
    v2->halfedge() = h2;
    v0->halfedge() = h_new2;
    v3->halfedge() = h5;

    e0->halfedge() = h0;
    e_new0->halfedge() = h_new2;
    e_new0->isNew = is_set_new;

    // later use
    e_reserve = e_new0;

    e_new1->halfedge() = h_new0;
    e_new1->isNew = is_set_new;
    e_new2->halfedge() = h_new4;
    e_new2->isNew = is_set_new;

    f0->halfedge() = h0;
    f1->halfedge() = h3;
    f_new0->halfedge() = h_new1;
    f_new1->halfedge() = h_new3;

    return v_new0;
  } 

  VertexIter HalfedgeMesh::splitEdge( EdgeIter e0 )
  {
    // TODO Part 5.
    // This method should split the given edge and return an iterator to the newly inserted vertex.
    // The halfedge of this vertex should point along the edge that was split, rather than the new edges.
    EdgeIter dummy;
    return splitEdgeWithReserve(*this, e0, dummy, false);
  }

  // helper function
  bool is_flip_ok(const EdgeIter &e) {
    if (e->isBoundary()) return false;
    if (!e->isNew) return false;

    /*
     *
     * o ----------- n
     * |  \          |
     * |    \        |   => OK to flip 
     * |       e     |
     * |          \  |
     * n ----------- n
     * 
     * n ----------- *
     * |  \          |
     * |    \        |   => Not OK to flip 
     * |       e     |
     * |          \  |
     * * ----------- n
     * 
     * o ----------- n
     * |  \          |
     * |    \        |   => Not OK to flip 
     * |       e     |
     * |          \  |
     * o ----------- n
     * 
     */ 

    auto v0 = e->halfedge()->vertex();
    auto v1 = e->halfedge()->twin()->vertex();
    auto v2 = e->halfedge()->next()->next()->vertex();
    auto v3 = e->halfedge()->twin()->next()->next()->vertex();

    if (v3->isNew && v2->isNew && (v0->isNew ^ v1->isNew)) 
      return true;
    return false;
  }

  void MeshResampler::upsample( HalfedgeMesh& mesh )
  {
    // TODO Part 6.
    // This routine should increase the number of triangles in the mesh using Loop subdivision.
    // One possible solution is to break up the method as listed below.

    // 1. Compute new positions for all the vertices in the input mesh, using the Loop subdivision rule,
    // and store them in Vertex::newPosition. At this point, we also want to mark each vertex as being
    // a vertex of the original mesh.
    
    // 2. Compute the updated vertex positions associated with edges, and store it in Edge::newPosition.
    
    // 3. Split every edge in the mesh, in any order. For future reference, we're also going to store some
    // information about which subdivide edges come from splitting an edge in the original mesh, and which edges
    // are new, by setting the flat Edge::isNew. Note that in this loop, we only want to iterate over edges of
    // the original mesh---otherwise, we'll end up splitting edges that we just split (and the loop will never end!)
    
    // 4. Flip any new edge that connects an old and new vertex.

    // 5. Copy the new vertex positions into final Vertex::position.

    // step 1
    for (auto v = mesh.verticesBegin(); v != mesh.verticesEnd(); v++) {
      v->isNew = false;
      if (v->isBoundary()) {
        v->newPosition = 3.0 / 4.0 * v->position + 
            1.0 / 8.0 * v->halfedge()->twin()->vertex()->position + 
            1.0 / 8.0 * v->halfedge()->next()->next()->vertex()->position;
        continue;
      }
      double n = v->degree();
      double u = (n == 3.0) ? 3.0 / 16.0 : 3.0 / (8.0 * n);
      v->newPosition = (1 - n * u) * v->position;
      auto h = v->halfedge();
      // iterate neighbor vertices
      do {
        v->newPosition += u * h->twin()->vertex()->position;
        h = h->twin()->next();
      } while (h != v->halfedge());
    }


    // step 2
    for (auto e = mesh.edgesBegin(); e != mesh.edgesEnd(); e++) {
      e->isNew = false;
      if (e->isBoundary()) {
        e->newPosition = (e->halfedge()->vertex()->position 
            + e->halfedge()->twin()->vertex()->position) / 2.0;
        continue;
      }
      auto v0 = e->halfedge()->vertex();
      auto v1 = e->halfedge()->twin()->vertex();
      auto v2 = e->halfedge()->next()->next()->vertex();
      auto v3 = e->halfedge()->twin()->next()->next()->vertex();
      e->newPosition = (3.0 * (v0->position + v1->position) + (v2->position + v3->position)) / 8.0;
    }


    // step 3
    std::vector<EdgeIter> reserved_edges;
    for (auto e = mesh.edgesBegin(); e != mesh.edgesEnd(); e++) {
      if (e->isNew) continue;
      // split edge sets new edges all to be new, but some
      // of them are actually cut from old edges
      EdgeIter e_reserve;
      auto v_new = splitEdgeWithReserve(mesh, e, e_reserve);
      v_new->isNew = true;
      reserved_edges.emplace_back(std::move(e_reserve));
    }
    for (auto &e : reserved_edges) {
      e->isNew = false;
    }



    // step 4
    for (auto e = mesh.edgesBegin(); e != mesh.edgesEnd(); e++) {
      if (is_flip_ok(e)) 
        mesh.flipEdge(e);
    }
    for (auto &e : reserved_edges) {
      e->isNew = true;
    }

    // step 5 

    // old vertices
    for (auto v = mesh.verticesBegin(); v != mesh.verticesEnd(); v++) {
      if (!v->isNew) {
        v->position = v->newPosition;
      }
    }
    // new vertices on shared edges
    for (auto e = mesh.edgesBegin(); e != mesh.edgesEnd(); e++) {
      if (!e->isNew) {
        VertexIter v = e->halfedge()->vertex()->isNew ? 
            e->halfedge()->vertex() : e->halfedge()->twin()->vertex();
        v->position = e->newPosition;
      }
    }

  }
}
