#include "student_code.h"
#include "mutablePriorityQueue.h"

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
    return Vector3D();
  }

  EdgeIter HalfedgeMesh::flipEdge( EdgeIter e0 )
  {
    // TODO Part 4.
    // This method should flip the given edge and return an iterator to the flipped edge.
    return EdgeIter();
  }

  VertexIter HalfedgeMesh::splitEdge( EdgeIter e0 )
  {
    // TODO Part 5.
    // This method should split the given edge and return an iterator to the newly inserted vertex.
    // The halfedge of this vertex should point along the edge that was split, rather than the new edges.
    return VertexIter();
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

  }
}
