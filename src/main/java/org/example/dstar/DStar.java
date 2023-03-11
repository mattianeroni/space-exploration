package org.example.dstar;

import org.example.Vec2;

import java.util.*;


public class DStar
{

    /* An implementation of the D* star lite algorithm */

    public PriorityQueue<HeapNode> heap;       // The heap of next positions to visit
    public Set<Vec2> inconsistents;            // The set of inconsistent nodes
    public float km;                           // Accumulation factor
    public Vec2 source, goal, current;         // Starting and ending positions, and the current position where the robot is

    int[][] slam;         // The current knowledge of the environment the algorithm has
    float[][] rhs;        // The second level estimate of distance between nodes and goal
    float[][] g;          // The matrices of distances between nodes and goal


    public DStar (Vec2 source, Vec2 goal, int[][] slam)
    {
        this.source = source;
        this.goal = goal;
        this.current = source;
        this.slam = slam;
        this.km = 0;
        this.rhs = new float[slam.length][slam[0].length];
        this.g = new float[slam.length][slam[0].length];
        this.heap = new PriorityQueue<>();
        this.inconsistents = new HashSet<>();
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


    // Get the neighbours of a position
    public List<Vec2> getNeighbours (Vec2 position)
    {

        List<Vec2> neighbours = new ArrayList<>(8);

        for (int i = position.x - 1; i < position.x + 2; i++){
            for (int j = position.y - 1; j < position.y + 2; j++){

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
