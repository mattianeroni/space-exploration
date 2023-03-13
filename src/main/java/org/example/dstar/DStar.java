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
    //public List<Vec2> currentPath;             // The current shortest path from current to goal
    public boolean changed = false;            // A flag that keeps track of changes after the last movement


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

        rhs[goal.x][goal.y] = 0.0f;
        HeapNode hn = computeHeapNode(goal);
        inconsistents.put(goal, hn);
        heap.add(hn);
    }


    // The cost of moving from a position to a close position
    public float moveCost(Vec2 p1, Vec2 p2)
    {
        // Same position
        if (p1.equals(p2))
            return 0.0f;

        // Movement into obstacle
        if (slam[p2.x][p2.y] == 1)
            return Float.POSITIVE_INFINITY;

        // Vertical or horizontal movement
        if (p1.x == p2.x || p1.y == p2.y)
            return 1.0f;

        // Diagonal movement
        return 1.414f;
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
        // Verify if something has changed to increase the accumulation factor
        if (changed)
        {
            changed = false;
            km += moveCost(last, current);
            computePath();
        }



        // Move to the position that corresponds to the shortest path to the goal
        Vec2 nextNode = null;
        float minCost = Float.POSITIVE_INFINITY;

        for (Vec2 node : getNeighbours(current)) {
            float cost = moveCost(current, node);
            if (cost + g[node.x][node.y] < minCost) {
                minCost = cost + g[node.x][node.y];
                nextNode = node;
            }
        }
        last = current;
        current = nextNode;

    }


    // Compute the shortest path to the goal according to the current knowledge of the environment
    public void computePath()
    {
        while (
                heap.peek() != null &&
                (heap.peek().compareTo(computeHeapNode(current)) < 0  ||  rhs[current.x][current.y] > g[current.x][current.y])
        )
        {
            HeapNode node = heap.poll();
            Vec2 position = node.position;
            HeapNode newNode = computeHeapNode(position);

            if (node.compareTo(newNode) < 0) {

                heap.add(newNode);
                inconsistents.put(position, newNode);

            } else if (g[position.x][position.y] > rhs[position.x][position.y]){

                g[position.x][position.y] = rhs[position.x][position.y];
                inconsistents.remove(position);
                for (Vec2 i : getNeighbours(position))
                    updateVertex(i);

            } else {

                float g_old = g[position.x][position.y];
                g[position.x][position.y] = Float.POSITIVE_INFINITY;
                updateVertex(position);
                for (Vec2 i : getNeighbours(position))
                    updateVertex(i);

            }
        }
    }


    public List<Vec2> extractPath()
    {
        List<Vec2> path = new ArrayList<>();
        path.add(current);
        Vec2 cNode = current;

        while (cNode != goal)
        {
            Vec2 minNode = null;
            float minCost = Float.POSITIVE_INFINITY;
            for (Vec2 next : getNeighbours(cNode)){
                if (g[next.x][next.y] < minCost){
                    minCost = g[next.x][next.y];
                    minNode = next;
                }
            }
            path.add(minNode);
            cNode = minNode;
        }

        path.add(goal);
        return path;
    }


    // Add a new obstacle to the slam
    public void addObstacle (Vec2 position)
    {
        changed = true;
        slam[position.x][position.y] = 1;
        g[position.x][position.y] = Float.POSITIVE_INFINITY;
        rhs[position.x][position.y] = Float.POSITIVE_INFINITY;
        for (Vec2 pos : getNeighbours(current))
            updateVertex(pos);
    }


    // Remove an obstacle from the slam
    public void removeObstacle (Vec2 position)
    {
        changed = true;
        slam[position.x][position.y] = 0;
        updateVertex(position);
        for (Vec2 pos : getNeighbours(current))
            updateVertex(pos);
    }


    public void updateVertex (Vec2 position)
    {
        int x = position.x, y = position.y;

        // Update the rhs value according to the neighbours
        if (!position.equals(goal))
        {
            rhs[x][y] = Float.POSITIVE_INFINITY;
            for (Vec2 n : getNeighbours(position))
                rhs[x][y] = Math.min(rhs[x][y], g[n.x][n.y] + moveCost(position, n));

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
