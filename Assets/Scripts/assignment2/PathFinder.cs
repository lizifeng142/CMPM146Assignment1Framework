using UnityEngine;
using System.Collections.Generic;

public class PathFinder : MonoBehaviour
{

    // Assignment 2: Implement AStar
    //
    // DO NOT CHANGE THIS SIGNATURE (parameter types + return type)
    // AStar will be given the start node, destination node and the target position, and should return 
    // a path as a list of positions the agent has to traverse to reach its destination, as well as the
    // number of nodes that were expanded to find this path
    // The last entry of the path will be the target position, and you can also use it to calculate the heuristic
    // value of nodes you add to your search frontier; the number of expanded nodes tells us if your search was
    // efficient
    //
    // Take a look at StandaloneTests.cs for some test cases
    // A* Pathfinding Algorithm Implementation
    // Returns a list of midpoints to follow and the number of expanded nodes
    public static (List<Vector3>, int) AStar(GraphNode start, GraphNode destination, Vector3 target)
    {
        var openSet = new List<AStarEntry>(); // Priority queue of nodes to be evaluated
        var cameFrom = new Dictionary<int, AStarEntry>(); // Maps node ID to its path entry
        var gScore = new Dictionary<int, float>(); // Cost from start to a given node
        var closedSet = new HashSet<int>(); // Set of evaluated nodes

        int expanded = 0; // Counter for how many nodes have been expanded

        // Initialize the start node
        AStarEntry startEntry = new AStarEntry(start, null, null, 0f, Vector3.Distance(start.GetCenter(), target));
        openSet.Add(startEntry);
        gScore[start.GetID()] = 0f;

        while (openSet.Count > 0)
        {
            // Sort the openSet by total cost (g + h), lowest first
            openSet.Sort((a, b) => (a.g + a.h).CompareTo(b.g + b.h));
            var current = openSet[0];
            openSet.RemoveAt(0);

            // If we reached the destination, reconstruct the path
            if (current.node.GetID() == destination.GetID())
            {
                var path = new List<Vector3>();
                while (current.wall != null)
                {
                    path.Insert(0, current.wall.midpoint);
                    current = current.parent;
                }
                path.Add(target); // Final point in the destination polygon
                return (path, expanded);
            }

            closedSet.Add(current.node.GetID());
            expanded++;

            // Check each neighbor of the current node
            foreach (var neighbor in current.node.GetNeighbors())
            {
                var neighborNode = neighbor.GetNode();
                int neighborId = neighborNode.GetID();
                if (closedSet.Contains(neighborId)) continue; // Skip if already evaluated

                // Calculate tentative cost to neighbor
                float tentativeG = current.g + Vector3.Distance(current.node.GetCenter(), neighborNode.GetCenter());
                bool inOpen = gScore.ContainsKey(neighborId);

                // If this path to neighbor is better, or neighbor hasn't been seen yet
                if (!inOpen || tentativeG < gScore[neighborId])
                {
                    gScore[neighborId] = tentativeG;
                    float h = Vector3.Distance(neighborNode.GetCenter(), target);
                    var entry = new AStarEntry(neighborNode, current, neighbor.GetWall(), tentativeG, h);

                    if (!inOpen) openSet.Add(entry);
                    cameFrom[neighborId] = entry;
                }
            }
        }

        // Return a fallback path if no path was found
        return (new List<Vector3> { target }, expanded);
    }

    // Helper class to track each entry in the priority queue
    class AStarEntry
    {
        public GraphNode node;
        public AStarEntry parent;
        public Wall wall;
        public float g;
        public float h;

        public AStarEntry(GraphNode node, AStarEntry parent, Wall wall, float g, float h)
        {
            this.node = node;
            this.parent = parent;
            this.wall = wall;
            this.g = g;
            this.h = h;
        }
    }

    public Graph graph;

    // Register for events
    void Start()
    {
        EventBus.OnTarget += PathFind;
        EventBus.OnSetGraph += SetGraph;
    }

    void Update() { }

    // Called when the graph is updated
    public void SetGraph(Graph g)
    {
        graph = g;
    }

    // Triggered when the user selects a target
    public void PathFind(Vector3 target)
    {
        if (graph == null) return;

        GraphNode start = null;
        GraphNode destination = null;

        // Find the start and destination nodes from the car position and target position
        foreach (var n in graph.all_nodes)
        {
            if (Util.PointInPolygon(transform.position, n.GetPolygon()))
                start = n;
            if (Util.PointInPolygon(target, n.GetPolygon()))
                destination = n;
        }

        // Proceed only if destination node is valid
        if (destination != null)
        {
            EventBus.ShowTarget(target);
            (List<Vector3> path, int expanded) = AStar(start, destination, target);

            Debug.Log("found path of length " + path.Count + " expanded " + expanded + " nodes, out of: " + graph.all_nodes.Count);
            EventBus.SetPath(path);
        }
    }
}
