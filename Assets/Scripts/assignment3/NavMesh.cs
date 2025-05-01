using System.Collections.Generic;
using System.Linq;
using UnityEngine;
using UnityEngine.InputSystem;
///
public class NavMesh : MonoBehaviour
{
    public Graph MakeNavMesh(List<Wall> outline)
    {
        Graph g = new Graph();
        g.all_nodes = new List<GraphNode>();
        List<Vector3> points = ExtractPoints(outline);
        List<List<Vector3>> convexPolys = SplitToConvex(points);

        foreach (var poly in convexPolys)
        {
            List<Wall> walls = new List<Wall>();
            for (int i = 0; i < poly.Count; i++)
                walls.Add(new Wall(poly[i], poly[(i + 1) % poly.Count]));
            g.all_nodes.Add(new GraphNode(g.all_nodes.Count, walls));
        }

        for (int i = 0; i < g.all_nodes.Count; i++)
        {
            for (int j = i + 1; j < g.all_nodes.Count; j++)
            {
                int edgeIndex;
                if (ShareEdge(g.all_nodes[i], g.all_nodes[j], out edgeIndex))
                {
                    g.all_nodes[i].AddNeighbor(g.all_nodes[j], edgeIndex);
                    g.all_nodes[j].AddNeighbor(g.all_nodes[i], edgeIndex);
                }
            }
        }

        return g;
    }

    List<Vector3> ExtractPoints(List<Wall> walls)
    {
        List<Vector3> pts = new List<Vector3>();
        foreach (var w in walls) pts.Add(w.start);
        return pts;
    }

    bool IsReflex(Vector3 prev, Vector3 curr, Vector3 next)
    {
        Vector3 v1 = curr - prev, v2 = next - curr;
        return Vector3.Cross(v1, v2).z < 0;
    }

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

    List<List<Vector3>> SplitToConvex(List<Vector3> poly)
    {
        if (IsConvex(poly)) return new List<List<Vector3>> { poly };
        for (int i = 0; i < poly.Count; i++)
        {
            Vector3 prev = poly[(i - 1 + poly.Count) % poly.Count];
            Vector3 curr = poly[i];
            Vector3 next = poly[(i + 1) % poly.Count];
            if (!IsReflex(prev, curr, next)) continue;

            int bestJ = -1;
            float bestDist = float.MinValue;
            for (int j = 0; j < poly.Count; j++)
            {
                if (j == i || Mathf.Abs(i - j) == 1 || Mathf.Abs(i - j) == poly.Count - 1)
                    continue;

                if (!ValidDiagonal(poly, i, j)) continue;

                float dist = (poly[i] - poly[j]).sqrMagnitude;
                if (dist > bestDist)
                {
                    bestDist = dist;
                    bestJ = j;
                }
            }

            if (bestJ != -1)
            {
                var p1 = GetSubPoly(poly, i, bestJ);
                var p2 = GetSubPoly(poly, bestJ, i);
                var result = new List<List<Vector3>>();
                result.AddRange(SplitToConvex(p1));
                result.AddRange(SplitToConvex(p2));
                return result;
            }
        }

        return new List<List<Vector3>> { poly };
    }

    List<Vector3> GetSubPoly(List<Vector3> poly, int start, int end)
    {
        List<Vector3> sub = new List<Vector3>();
        for (int i = start; i != end; i = (i + 1) % poly.Count)
            sub.Add(poly[i]);
        sub.Add(poly[end]);
        return sub;
    }

    bool ValidDiagonal(List<Vector3> poly, int i, int j)
    {
        Wall diag = new Wall(poly[i], poly[j]);
        for (int k = 0; k < poly.Count; k++)
        {
            Vector3 a = poly[k], b = poly[(k + 1) % poly.Count];
            if ((i == k && j == (k + 1) % poly.Count) || (j == k && i == (k + 1) % poly.Count)) continue;
            if (diag.Crosses(a, b)) return false;
        }
        return true;
    }

    bool ShareEdge(GraphNode a, GraphNode b, out int edgeIdx)
    {
        List<Wall> aEdges = a.GetPolygon(), bEdges = b.GetPolygon();
        for (int i = 0; i < aEdges.Count; i++)
            foreach (var w in bEdges)
                if (aEdges[i].Same(w)) { edgeIdx = i; return true; }
        edgeIdx = -1;
        return false;
    }

    void Start()
    {
        EventBus.OnSetMap += SetMap;
    }

    void Update() { }

    public void SetMap(List<Wall> outline)
    {
        Graph navmesh = MakeNavMesh(outline);
        if (navmesh != null)
        {
            Debug.Log("got navmesh: " + navmesh.all_nodes.Count);
            EventBus.SetGraph(navmesh);
        }
    }
}