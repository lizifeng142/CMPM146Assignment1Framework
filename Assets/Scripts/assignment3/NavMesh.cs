using System.Collections.Generic;
using System.Linq;
using UnityEngine;

public class NavMesh : MonoBehaviour
{
    public Graph MakeNavMesh(List<Wall> outline)
    {
        Graph g = new Graph { outline = outline, all_nodes = new List<GraphNode>() };

        List<Vector3> verts = outline.Select(w => w.start).ToList();
        if (IsClockwise(verts)) verts.Reverse(); // Ensure counter-clockwise winding

        List<List<Vector3>> convexPolys = SplitToConvex(verts);
        int id = 0;
        foreach (var poly in convexPolys)
        {
            var walls = new List<Wall>();
            for (int i = 0; i < poly.Count; i++)
                walls.Add(new Wall(poly[i], poly[(i + 1) % poly.Count]));
            g.all_nodes.Add(new GraphNode(id++, walls));
        }

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

    // Recursively splits a polygon into convex sub-polygons using Strategy 1
    List<List<Vector3>> SplitToConvex(List<Vector3> poly)
{
    if (IsConvex(poly)) return new List<List<Vector3>> { poly };

    for (int i = 0; i < poly.Count; i++)
    {
        Vector3 prev = poly[(i - 1 + poly.Count) % poly.Count];
        Vector3 curr = poly[i];
        Vector3 next = poly[(i + 1) % poly.Count];

        if (!IsReflex(prev, curr, next)) continue;

        // Try connecting to the next available vertex in order, skipping ahead 3+
        for (int offset = 3; offset < poly.Count - 1; offset++)
        {
            int j = (i + offset) % poly.Count;

            // Skip adjacent or overlapping vertices
            if (j == i || Mathf.Abs(i - j) <= 1 || Mathf.Abs(i - j) == poly.Count - 1) continue;
            if (!ValidDiagonal(poly, i, j)) continue;

            // Found a good diagonal
            var p1 = GetSubPoly(poly, i, j);
            var p2 = GetSubPoly(poly, j, i);

            if (IsClockwise(p1)) p1.Reverse();
            if (IsClockwise(p2)) p2.Reverse();

            Debug.DrawLine(poly[i] + Vector3.up * 2, poly[j] + Vector3.up * 2, Color.yellow, 20f);

            var result = new List<List<Vector3>>();
            result.AddRange(SplitToConvex(p1));
            result.AddRange(SplitToConvex(p2));
            return result;
        }
    }

    Debug.LogWarning("⚠️ Failed to split non-convex polygon.");
    return new List<List<Vector3>> { poly };
}

    // Extracts a sub-polygon from start to end (inclusive), wrapping around
    List<Vector3> GetSubPoly(List<Vector3> poly, int start, int end)
    {
        List<Vector3> sub = new();
        for (int i = start; i != end; i = (i + 1) % poly.Count)
            sub.Add(poly[i]);
        sub.Add(poly[end]);
        return sub;
    }

    // Checks if the angle at 'curr' is a reflex (i.e., > 180 degrees)
    bool IsReflex(Vector3 prev, Vector3 curr, Vector3 next)
    {
        Vector3 toPrev = curr - prev;
        Vector3 toNext = next - curr;
        float crossY = Vector3.Cross(toPrev, toNext).y;
        return crossY < 0;
    }

    // Returns true if the polygon is convex (has no reflex vertices)
    bool IsConvex(List<Vector3> poly)
    {
        for (int i = 0; i < poly.Count; i++)
        {
            Vector3 prev = poly[(i - 1 + poly.Count) % poly.Count];
            Vector3 curr = poly[i];
            Vector3 next = poly[(i + 1) % poly.Count];
            if (IsReflex(prev, curr, next)) return false;
        }
        return true;
    }

    // Checks if a polygon is ordered clockwise (used for correction)
    bool IsClockwise(List<Vector3> pts)
    {
        float sum = 0;
        for (int i = 0; i < pts.Count; i++)
        {
            Vector3 current = pts[i];
            Vector3 next = pts[(i + 1) % pts.Count];
            sum += (next.x - current.x) * (next.z + current.z);
        }
        return sum > 0;
    }

    // Determines whether point 'p' is inside a polygon
    bool PointInPolygon(Vector3 p, List<Vector3> poly)
    {
        int count = 0;
        for (int i = 0; i < poly.Count; i++)
        {
            Vector3 a = poly[i], b = poly[(i + 1) % poly.Count];
            if ((a.z > p.z) != (b.z > p.z))
            {
                float t = (p.z - a.z) / (b.z - a.z);
                float x = a.x + t * (b.x - a.x);
                if (p.x < x) count++;
            }
        }
        return count % 2 == 1;
    }

    // Checks if a diagonal between i and j is valid (inside polygon and non-intersecting)
    bool ValidDiagonal(List<Vector3> poly, int i, int j)
    {
        Wall diag = new(poly[i], poly[j]);
        Vector3 mid = (poly[i] + poly[j]) * 0.5f;
        if (!PointInPolygon(mid, poly)) return false;

        for (int k = 0; k < poly.Count; k++)
        {
            Vector3 a = poly[k], b = poly[(k + 1) % poly.Count];
            if ((a == poly[i] || a == poly[j] || b == poly[i] || b == poly[j])) continue;
            if (diag.Crosses(a, b)) return false;
        }
        return true;
    }

    // Checks if two polygons share a wall
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

    public void SetMap(List<Wall> outline)
    {
        Graph navmesh = MakeNavMesh(outline);
        Debug.Log("Got navmesh with " + navmesh.all_nodes.Count + " nodes.");
        EventBus.SetGraph(navmesh);
    }
}