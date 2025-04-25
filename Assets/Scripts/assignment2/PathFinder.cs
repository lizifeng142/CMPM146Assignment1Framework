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

    // A* pathfinding method that returns a path and the number of expanded nodes
    public static (List<Vector3>, int) AStar(GraphNode start, GraphNode destination, Vector3 target)
    {
        var open = new List<AStarEntry>();                      // List of nodes to evaluate
        var visited = new Dictionary<int, AStarEntry>();        // Tracks visited nodes and their best known paths
        int expanded = 0;                                       // Count of nodes expanded (used for debugging)

        // Create and add the starting node to the open list
        var startEntry = new AStarEntry
        {
            node = start,
            g = 0,                                               
            h = Vector3.Distance(start.GetCenter(), target),    
            wall = null,
            from = null
        };
        open.Add(startEntry);
        visited[start.GetID()] = startEntry;

        // Main loop: continue as long as there are nodes to explore
        while (open.Count > 0)
        {
            // Always expand the node with the lowest F value: f = g + h
            open.Sort((a, b) => (a.g + a.h).CompareTo(b.g + b.h));
            var current = open[0];
            open.RemoveAt(0);
            expanded++;

            // If destination is reached, reconstruct the path using wall midpoints
            if (current.node == destination)
            {
                List<Vector3> path = new List<Vector3>();
                var cursor = current;
                while (cursor.from != null && cursor.wall != null)
                {
                    path.Insert(0, cursor.wall.midpoint);
                    cursor = cursor.from;
                }
                path.Add(target); 
                return (path, expanded);
            }

            // Iterate through all neighbors of the current node
            foreach (var neighbor in current.node.GetNeighbors())
            {
                var neighborNode = neighbor.GetNode();
                int id = neighborNode.GetID();
                Vector3 toMidpoint = neighbor.GetWall().midpoint;

                // Determine distance from either the starting point or previous midpoint
                Vector3 fromPosition = current.wall == null ? toMidpoint : current.wall.midpoint;

                // g = cost to reach this neighbor, h = heuristic to target
                float tentativeG = current.g + Vector3.Distance(fromPosition, toMidpoint);
                float h = Vector3.Distance(toMidpoint, target);

                // If already visited with a lower g, skip this neighbor
                if (visited.ContainsKey(id) && tentativeG >= visited[id].g)
                    continue;

                // Otherwise, create a new path entry and add it to open/visited
                var entry = new AStarEntry
                {
                    node = neighborNode,
                    g = tentativeG,
                    h = h,
                    wall = neighbor.GetWall(),
                    from = current
                };

                visited[id] = entry;
                open.Add(entry);
            }
        }

        // If no path was found, return the target only
        return (new List<Vector3> { target }, expanded);
    }

    // Internal class to represent an entry in the A* search
    class AStarEntry
    {
        public GraphNode node;
        public float g;          // Cost from start to this node
        public float h;          // Heuristic cost from this node to goal
        public Wall wall;        // Wall used to reach this node
        public AStarEntry from;  // Previous node in the path
    }

    public Graph graph;

    // Subscribe to events on startup
    void Start()
    {
        EventBus.OnTarget += PathFind;
        EventBus.OnSetGraph += SetGraph;
    }

    // Callback for when a graph is set externally
    public void SetGraph(Graph g)
    {
        graph = g;
    }

    // Callback for when a target is clicked
    public void PathFind(Vector3 target)
    {
        if (graph == null) return;

        GraphNode start = null;
        GraphNode destination = null;

        // Find the graph node the player is in
        foreach (var n in graph.all_nodes)
        {
            if (Util.PointInPolygon(transform.position, n.GetPolygon()))
            {
                start = n;
            }
            if (Util.PointInPolygon(target, n.GetPolygon()))
            {
                destination = n;
            }
        }

        // If destination is valid, run A* and display path
        if (destination != null)
        {
            EventBus.ShowTarget(target);
            (List<Vector3> path, int expanded) = PathFinder.AStar(start, destination, target);

            Debug.Log("found path of length " + path.Count + " expanded " + expanded + " nodes, out of: " + graph.all_nodes.Count);
            EventBus.SetPath(path);
        }
    }
}
