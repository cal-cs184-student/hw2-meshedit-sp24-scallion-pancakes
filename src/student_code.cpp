#include "student_code.h"
#include "mutablePriorityQueue.h"

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
    vector<Vector2D> new_points = std::vector<Vector2D>();

    if (points.size() == 1) {
        new_points.push_back(points.at(0));
        return new_points;
    }
    for (int i=0; i < points.size() - 1; i++) {
        Vector2D lerp = (1 - this->t) * points.at(i) + this->t * points.at(i+1);
        new_points.push_back(lerp);
    }
    return new_points;
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
      vector<Vector3D> new_points = std::vector<Vector3D>();

      if (points.size() == 1) {
          new_points.push_back(points.at(0));
          return new_points;
      }
      for (int i=0; i < points.size() - 1; i++) {
          Vector3D lerp = (1 - t) * points.at(i) + t * points.at(i+1);
          new_points.push_back(lerp);
      }
      return new_points;
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
    vector<Vector3D> new_points = points;
    while (new_points.size() > 1) {
        new_points = evaluateStep(new_points, t);
    }

    return new_points.at(0);
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
      vector<Vector3D> intermediate_points = vector<Vector3D>();
      for (int r=0; r < controlPoints.size(); r++) {
          vector<Vector3D> row = controlPoints.at(r);
          intermediate_points.push_back(evaluate1D(row, u));
      }

      Vector3D final_point = evaluate1D(intermediate_points, v);
      return final_point;
  }

  Vector3D Vertex::normal( void ) const
  {
    // TODO Part 3.
    // Returns an approximate unit normal at this vertex, computed by
    // taking the area-weighted average of the normals of neighboring
    // triangles, then normalizing.

    Vector3D normal_sum = Vector3D();
    auto hedge = this->halfedge();


    do {
        auto hedge2 = hedge->next()->next(); // on the same face as hedge

        Vector3D v_out = this->position - hedge->next()->vertex()->position;
        Vector3D v_in = hedge2->vertex()->position - this->position;

        Vector3D normal = cross(v_in, v_out);

        normal_sum = normal_sum + normal;

        hedge = hedge->twin()->next();
    } while(hedge != this->halfedge());


    return normal_sum / normal_sum.norm();
  }

  EdgeIter HalfedgeMesh::flipEdge( EdgeIter e0 )
  {
    // TODO Part 4.
    // This method should flip the given edge and return an iterator to the flipped edge.
    // src: http://15462.courses.cs.cmu.edu/fall2015content/misc/HalfedgeEdgeOpImplementationGuide.pdf
    if (e0->isBoundary()) {
        return e0;
    }
    // collect half-edges
    HalfedgeIter h0 = e0->halfedge();
    HalfedgeIter h1 = h0->next();
    HalfedgeIter h2 = h1->next();
    HalfedgeIter h3 = h0->twin();
    HalfedgeIter h4 = h3->next();
    HalfedgeIter h5 = h4->next();
    HalfedgeIter h6 = h1->twin();
    HalfedgeIter h7 = h2->twin();
    HalfedgeIter h8 = h4->twin();
    HalfedgeIter h9 = h5->twin();
    // collect vertices
    VertexIter v0 = h0->vertex();
    VertexIter v1 = h3->vertex();
    VertexIter v2 = h6->vertex();
    VertexIter v3 = h8->vertex();
    //collect edges
    EdgeIter e1 = h1->edge();
    EdgeIter e2 = h2->edge();
    EdgeIter e3 = h4->edge();
    EdgeIter e4 = h5->edge();
    //collect faces
    FaceIter f0 = h0->face();
    FaceIter f1 = h3->face();

    // reassign half-edge
      h0->next() = h1;
      h0->twin() = h3;
      h0->vertex() = v3;
      h0->edge() = e0;
      h0->face() = f0;

      h1->next() = h2;
      h1->twin() = h7;
      h1->vertex() = v2;
      h1->edge() = e2;
      h1->face() = f0;

      h2->next() = h0;
      h2->twin() = h8;
      h2->vertex() = v0;
      h2->edge() = e3;
      h2->face() = f0;

      h3->next() = h4;
      h3->twin() = h0;
      h3->vertex() =v2;
      h3->edge() = e0;
      h3->face() = f1;

      h4->next() = h5;
      h4->twin() = h9;
      h4->vertex() = v3;
      h4->edge() = e4;
      h4->face() = f1;

      h5->next() = h3;
      h5->twin() = h6;
      h5->vertex() = v1;
      h5->edge() = e1;
      h5->face() = f1;

      h6->next() = h6->next();
      h6->twin() = h5;
      h6->vertex() = v2;
      h6->edge() = e1;
      h6->face() = h6->face();

      h7->next() = h7->next();
      h7->twin() = h1;
      h7->vertex() = v0;
      h7->edge() = e2;
      h7->face() = h7->face();

      h8->next() = h8->next();
      h8->twin() = h2;
      h8->vertex() = v3;
      h8->edge() = e3;
      h8->face() = h8->face();

      h9->next() = h9->next(); // didn’t change, but set it anyway!
      h9->twin() = h4;
      h9->vertex() = v1;
      h9->edge() = e4;
      h9->face() = h9->face(); // didn’t change, but set it anyway!

      // reassign vertices
      v0->halfedge() = h2;
      v1->halfedge() = h5;
      v2->halfedge() = h3;
      v3->halfedge() = h0;

      // reassign edges
      e0->halfedge() = h0;
      e1->halfedge() = h1;
      e2->halfedge() = h2;
      e3->halfedge() = h4;
      e4->halfedge() = h5;

      // reassign faces
      f0->halfedge() = h0;
      f1->halfedge() = h4;

      return e0;
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
