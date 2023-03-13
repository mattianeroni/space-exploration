package org.example.dstar;

import org.example.Vec2;

import java.util.*;


public class DStar
{

    /* An implementation of the D* star lite algorithm */

    public PriorityQueue<HeapNode> heap;       // The heap of next positions to visit
    public Map<Vec2, HeapNode> inconsistents;  // The set of map of inconsistent nodes (used only because
                                               // looking for an element in a hash map is computationally faster than
                                               // looking for it in the queue
    public float km;                           // Accumulation factor
    public Vec2 source, goal, current, last;   // Starting and ending positions, the current position where the robot is,
                                               // and the last position visited by the robot
    //public int viewSize = 3;                 // How far the robot can see

    int[][] slam;         // The current knowledge of the environment the algorithm has
    float[][] rhs;        // The second level estimate of distance between nodes and goal
    float[][] g;          // The matrices of distances between nodes and goal


    public DStar (Vec2 source, Vec2 goal, int[][] slam)
    {
        this.source = source;
        this.goal = goal;
        this.current = source;
        this.last = null;
        this.slam = slam;
        this.km = 0;
        this.heap = new PriorityQueue<>();
        this.inconsistents = new HashMap<>();

        this.rhs = new float[slam.length][slam[0].length];
        this.g = new float[slam.length][slam[0].length];

        for (int i = 0; i < slam.length; i++)
        {
            Arrays.fill(g[i], Float.POSITIVE_INFINITY);
            Arrays.fill(rhs[i], Float.POSITIVE_INFINITY);
        }

        rhs[source.x][source.y] = 0;
    }


    // Compute the heap node and the priority associated with a position
    public HeapNode computeHeapNode (Vec2 position)
    {
        int x = position.x, y = position.y;
        return new HeapNode(
                Math.min(g[x][y], rhs[x][y]) + km + euclidean(source, position),
                Math.min(g[x][y], rhs[x][y]),
                position
        );
    }


    // Make a step to the goal
    public void step ()
    {
        // Move to the position that corresponds to the shortest path to the goal
        List<Vec2> nextNodes = getNeighbours(current);
        Vec2 nextNode = null;
        float minCost = Float.POSITIVE_INFINITY;

        for (Vec2 node : nextNodes) {
            float cost = (node.x == current.x || node.y == current.y) ? 1.0f : 1.414f;
            if (cost + g[node.x][node.y] < minCost) {
                minCost = cost + g[node.x][node.y];
                nextNode = node;
            }
        }

        current = nextNode;
    }


    // Add a new obstacle to the slam
    public void addObstacle (Vec2 position)
    {
        slam[position.x][position.y] = 1;
        g[position.x][position.y] = Float.POSITIVE_INFINITY;
        rhs[position.x][position.y] = Float.POSITIVE_INFINITY;
        for (Vec2 pos : getNeighbours(current))
            updateVertex(pos);
    }


    // Remove an obstacle from the slam
    public void removeObstacle (Vec2 position)
    {
        slam[position.x][position.y] = 0;
        updateVertex(position);
        for (Vec2 pos : getNeighbours(current))
            updateVertex(pos);
    }


    public void updateVertex(Vec2 position)
    {
        int x = position.x, y = position.y;

        // Update the rhs value according to the neighbours
        if (!position.equals(goal))
        {
            rhs[x][y] = Float.POSITIVE_INFINITY;
            for (Vec2 n : getNeighbours(position)) {
                float cost = (n.x == x || n.y == y) ? 1.0f : 1.414f;
                rhs[x][y] = Math.min(rhs[x][y], g[n.x][n.y] + cost);
            }
        }

        // Remove from the heap (it will be re-added with updated costs if inconsistent - see next lines)
        if (inconsistents.containsKey(position))
        {
            HeapNode node = inconsistents.get(position);
            inconsistents.remove(position);
            heap.remove(node);
        }

        // Add to the heap if inconsistent
        if (g[x][y] != rhs[x][y])
        {
            HeapNode node = computeHeapNode(position);
            inconsistents.put(position, node);
            heap.add(node);
        }

    }


    // Get the neighbours of a position
    public List<Vec2> getNeighbours (Vec2 position)
    {
        List<Vec2> neighbours = new ArrayList<>(8);

        for (int i = position.x - 1; i < position.x + 2; i++)
        {
            for (int j = position.y - 1; j < position.y + 2; j++)
            {

                if (i == position.x && j == position.y)
                    continue;

                if (i >= 0 && i < slam.length && j >= 0 && j < slam[0].length)
                    neighbours.add(new Vec2(i, j));

            }
        }

        return neighbours;
    }


    // The euclidean distance between two positions
    public static float euclidean (Vec2 v1, Vec2 v2)
    {
        return (float) Math.sqrt(Math.pow(v1.x - v2.x, 2) + Math.pow(v1.y - v2.y, 2));
    }

}
