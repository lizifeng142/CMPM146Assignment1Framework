using System.Collections.Generic;
using System.Linq;
using UnityEngine;

public class NavMesh : MonoBehaviour
{
    // Builds the navigation mesh graph from an outer polygon outline
    public Graph MakeNavMesh(List<Wall> outline)
    {
        Graph g = new Graph { outline = outline, all_nodes = new List<GraphNode>() };

        // Extract vertices from wall start points
        List<Vector3> verts = outline.Select(w => w.start).ToList();

        // Ensure the polygon is counter-clockwise
        if (IsClockwise(verts)) verts.Reverse();

        // Split the non-convex polygon into a list of convex sub-polygons
        List<List<Vector3>> convexPolys = SplitToConvex(verts);
        int id = 0;

        // Convert each convex polygon into a GraphNode with walls
        foreach (var poly in convexPolys)
        {
            var walls = new List<Wall>();
            for (int i = 0; i < poly.Count; i++)
                walls.Add(new Wall(poly[i], poly[(i + 1) % poly.Count]));
            g.all_nodes.Add(new GraphNode(id++, walls));
        }

        // Connect nodes by identifying shared edges
        for (int i = 0; i < g.all_nodes.Count; i++)
        {
            for (int j = i + 1; j < g.all_nodes.Count; j++)
            {
                if (TryGetSharedEdge(g.all_nodes[i], g.all_nodes[j], out int edgeA, out int edgeB))
                {
                    g.all_nodes[i].AddNeighbor(g.all_nodes[j], edgeA);
                    g.all_nodes[j].AddNeighbor(g.all_nodes[i], edgeB);
                }
            }
        }

        return g;
    }

    // Recursively splits a polygon into convex sub-polygons using valid diagonals
    List<List<Vector3>> SplitToConvex(List<Vector3> polygon)
    {
        var result = new List<List<Vector3>>();
        var stack = new Stack<List<Vector3>>();
        stack.Push(polygon);

        while (stack.Count > 0)
        {
            var poly = stack.Pop();

            if (IsConvex(poly))
            {
                result.Add(poly);
                continue;
            }

            bool splitOccurred = false;

            // Try to find a reflex vertex to split from
            for (int i = 0; i < poly.Count; i++)
            {
                Vector3 prev = poly[(i - 1 + poly.Count) % poly.Count];
                Vector3 curr = poly[i];
                Vector3 next = poly[(i + 1) % poly.Count];

                if (!IsReflex(prev, curr, next))
                    continue;

                // Try to find a valid diagonal from reflex vertex
                for (int offset = 2; offset < poly.Count - 1; offset++)
                {
                    int j = (i + offset) % poly.Count;

                   
                    if (j == i || Mathf.Abs(i - j) <= 1 || Mathf.Abs(i - j) == poly.Count - 1)
                        continue;

                    if (!ValidDiagonal(poly, i, j))
                        continue;

                    
                    var p1 = GetSubPoly(poly, i, j);
                    var p2 = GetSubPoly(poly, j, i);

                    if (IsClockwise(p1)) p1.Reverse();
                    if (IsClockwise(p2)) p2.Reverse();

                    Debug.DrawLine(poly[i] + Vector3.up * 2, poly[j] + Vector3.up * 2, Color.yellow, 30f);

                    stack.Push(p1);
                    stack.Push(p2);
                    splitOccurred = true;
                    break;
                }

                if (splitOccurred) break;
            }


            if (!splitOccurred)
            {
                Debug.LogWarning("Polygon could not be split but is not convex!");
                result.Add(poly);
            }
        }

        return result;
    }

    // Returns a portion of the polygon from start to end (in order)
    List<Vector3> GetSubPoly(List<Vector3> poly, int start, int end)
    {
        List<Vector3> sub = new();
        for (int i = start; i != end; i = (i + 1) % poly.Count)
            sub.Add(poly[i]);
        sub.Add(poly[end]);
        return sub;
    }

    // Checks if angle formed at curr is reflex (i.e. > 180 degrees)
    bool IsReflex(Vector3 prev, Vector3 curr, Vector3 next)
    {
        Vector2 vPrev = new Vector2(prev.x, prev.z);
        Vector2 vCurr = new Vector2(curr.x, curr.z);
        Vector2 vNext = new Vector2(next.x, next.z);

        Vector2 a = vCurr - vPrev;
        Vector2 b = vNext - vCurr;

        float cross = a.x * b.y - a.y * b.x;
        return cross < 0;
    }

    // Wrapper to check reflex condition on a vertex index
    bool IsReflexVertex(List<Vector3> poly, int i)
    {
        Vector3 prev = poly[(i - 1 + poly.Count) % poly.Count];
        Vector3 curr = poly[i];
        Vector3 next = poly[(i + 1) % poly.Count];
        return IsReflex(prev, curr, next);
    }

    // Checks if a polygon is fully convex
    bool IsConvex(List<Vector3> poly)
    {
        for (int i = 0; i < poly.Count; i++)
        {
            if (IsReflexVertex(poly, i)) return false;
        }
        return true;
    }

    bool IsClockwise(List<Vector3> pts)
    {
        float area = 0;
        for (int i = 0; i < pts.Count; i++)
        {
            Vector2 curr = new(pts[i].x, pts[i].z);
            Vector2 next = new(pts[(i + 1) % pts.Count].x, pts[(i + 1) % pts.Count].z);
            area += (next.x - curr.x) * (next.y + curr.y);
        }
        return area > 0;
    }

    // Determines if a point is inside a polygon
    bool PointInPolygon(Vector3 p, List<Vector3> poly)
    {
        int count = 0;
        Vector2 point = new(p.x, p.z);
        for (int i = 0; i < poly.Count; i++)
        {
            Vector2 a = new(poly[i].x, poly[i].z);
            Vector2 b = new(poly[(i + 1) % poly.Count].x, poly[(i + 1) % poly.Count].z);
            if ((a.y > point.y) != (b.y > point.y))
            {
                float t = (point.y - a.y) / (b.y - a.y);
                float x = a.x + t * (b.x - a.x);
                if (point.x < x) count++;
            }
        }
        return count % 2 == 1;
    }

   
    bool ApproximatelyEqual(Vector3 a, Vector3 b)
    {
        return (a - b).sqrMagnitude < 1e-6f;
    }

    // Checks if a diagonal from i to j is valid (inside polygon and not intersecting other edges)
    bool ValidDiagonal(List<Vector3> poly, int i, int j)
    {
        Wall diag = new(poly[i], poly[j]);
        Vector3 mid = (poly[i] + poly[j]) * 0.5f;
        if (!PointInPolygon(mid, poly)) return false;

        for (int k = 0; k < poly.Count; k++)
        {
            Vector3 a = poly[k];
            Vector3 b = poly[(k + 1) % poly.Count];

            if (ApproximatelyEqual(a, poly[i]) || ApproximatelyEqual(a, poly[j]) ||
                ApproximatelyEqual(b, poly[i]) || ApproximatelyEqual(b, poly[j]))
                continue;

            if (diag.Crosses(a, b)) return false;
        }
        return true;
    }

    // Checks if two polygons share an identical edge and returns the edge indices
    bool TryGetSharedEdge(GraphNode a, GraphNode b, out int edgeA, out int edgeB)
    {
        var aEdges = a.GetPolygon();
        var bEdges = b.GetPolygon();
        for (int i = 0; i < aEdges.Count; i++)
        {
            for (int j = 0; j < bEdges.Count; j++)
            {
                if (aEdges[i].Same(bEdges[j]))
                {
                    edgeA = i;
                    edgeB = j;
                    return true;
                }
            }
        }
        edgeA = edgeB = -1;
        return false;
    }

   
    void Start() => EventBus.OnSetMap += SetMap;

    void Update() { }

    // Called when a new map is set; builds the graph and notifies the system
    public void SetMap(List<Wall> outline)
    {
        Graph navmesh = MakeNavMesh(outline);
        Debug.Log("Got navmesh with " + navmesh.all_nodes.Count + " nodes.");
        EventBus.SetGraph(navmesh);
    }
}
