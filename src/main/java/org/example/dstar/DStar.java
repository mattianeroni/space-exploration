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
    public boolean changed = false;            // A flag that says if new obstacles have been found
    public Set<Vec2> covered;                  // The set of positions covered by the robot

    public int[][] slam;                       // The current knowledge of the environment the algorithm has
    public float[][] rhs;                      // The second level estimate of distance between nodes and goal
    public float[][] g;                        // The matrices of distances between nodes and goal

    public DStar (Vec2 source, Vec2 goal, int[][] slam)
    {
        this.source = source;
        this.goal = goal;
        this.current = source;
        this.last = source;
        this.slam = slam;
        this.km = 0.0f;
        this.heap = new PriorityQueue<>();
        this.inconsistents = new HashMap<>();
        this.covered = new HashSet<>();

        // Init matrices for distances estimation and SLAM update
        this.rhs = new float[slam.length][slam[0].length];
        this.g = new float[slam.length][slam[0].length];
        for (int i = 0; i < slam.length; i++)
        {
            Arrays.fill(g[i], Float.POSITIVE_INFINITY);
            Arrays.fill(rhs[i], Float.POSITIVE_INFINITY);
        }

        // Init heap and matrices
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

        // Move into obstacle or from obstacle
        if ( slam[p2.x][p2.y] == 1 )
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
        // Arrived
        if (current.equals(goal))
            return;

        // No path to goal found
        if (rhs[current.x][current.y] == Float.POSITIVE_INFINITY) {
            System.out.println("[WARNING] No path found.");
            return;
        }

        // Move to the next position
        Vec2 nextNode = null;
        float minCost = Float.POSITIVE_INFINITY;

        for (Vec2 node : getNeighbours(current))
        {
            float cost = moveCost(current, node);
            if (cost + g[node.x][node.y] < minCost)
            {
                minCost = cost + g[node.x][node.y];
                nextNode = node;
            }
        }
        current = nextNode;
        covered.add(current);

    }


    // Update a vertex
    public void updateVertex (Vec2 position)
    {
        int x = position.x, y = position.y;

        if (g[x][y] != rhs[x][y] && inconsistents.containsKey(position))
        {

            HeapNode node = inconsistents.get(position);
            HeapNode updatedNode = computeHeapNode(position);
            inconsistents.put(position, updatedNode);
            heap.remove(node);
            heap.add(updatedNode);

        } else if (g[x][y] != rhs[x][y] && !inconsistents.containsKey(position))
        {

            HeapNode node = computeHeapNode(position);
            inconsistents.put(position, node);
            heap.add(node);

        } else if (g[x][y] == rhs[x][y] && inconsistents.containsKey(position))
        {

            HeapNode node = inconsistents.get(position);
            inconsistents.remove(position);
            heap.remove(node);

        }

    }


    public void updateSlam (int x, int y, int value)
    {
        Vec2 changedNode = new Vec2(x, y);
        this.changed = true;

        // Iterate the neighbours
        for (Vec2 neighbour : getNeighbours(changedNode))
        {
            // From changed cell to neighbours
            Vec2 u = changedNode, v = neighbour;
            float c_old = moveCost(u, v);
            float c_old_reverse = moveCost(v, u);
            slam[changedNode.x][changedNode.y] = value;

            if (c_old > moveCost(u,v) && !u.equals(goal))
            {
                rhs[u.x][u.y] = Math.min(rhs[u.x][u.y], moveCost(u, v) + g[v.x][v.y]);
            }
            else if (rhs[u.x][u.y] == c_old + g[v.x][v.y] && !u.equals(goal))
            {
                rhs[u.x][u.y] = Float.POSITIVE_INFINITY;
                for (Vec2 succ : getNeighbours(u))
                    rhs[u.x][u.y] = Math.min(rhs[u.x][u.y], moveCost(u, succ) + g[succ.x][succ.y]);
            }
            updateVertex(u);

            // From neighbours to changed cell
            u = neighbour; v = changedNode;
            if (c_old_reverse > moveCost(u,v) && !u.equals(goal))
            {
                rhs[u.x][u.y] = Math.min(rhs[u.x][u.y], moveCost(u, v) + g[v.x][v.y]);
            }
            else if (rhs[u.x][u.y] == c_old_reverse + g[v.x][v.y] && !u.equals(goal))
            {
                rhs[u.x][u.y] = Float.POSITIVE_INFINITY;
                for (Vec2 succ : getNeighbours(u))
                    rhs[u.x][u.y] = Math.min(rhs[u.x][u.y], moveCost(u, succ) + g[succ.x][succ.y]);
            }
            updateVertex(u);

        }
    }


    // Compute the shortest path to the goal according to the current knowledge of the environment
    public void computePath()
    {
        if (changed)
        {
            changed = false;
            km += moveCost(last, current);
            last = current;
        }

        while (
                heap.peek() != null &&
                (heap.peek().compareTo(computeHeapNode(current)) < 0  ||
                rhs[current.x][current.y] > g[current.x][current.y])
        )
        {
            HeapNode node = heap.poll();
            Vec2 position = node.position;
            HeapNode newNode = computeHeapNode(position);

            if (node.compareTo(newNode) < 0)
            {

                heap.add(newNode);
                inconsistents.put(position, newNode);

            } else if (g[position.x][position.y] > rhs[position.x][position.y])
            {

                g[position.x][position.y] = rhs[position.x][position.y];
                inconsistents.remove(position);
                for (Vec2 s : getNeighbours(position)) {
                    if (!s.equals(goal))
                        rhs[s.x][s.y] = Math.min( rhs[s.x][s.y], moveCost(s, position) + g[position.x][position.y] );
                    updateVertex(s);
                }

            } else
            {
                float g_old = g[position.x][position.y];
                g[position.x][position.y] = Float.POSITIVE_INFINITY;
                List<Vec2> pred = getNeighbours(position);
                pred.add(position);
                for (Vec2 s : pred)
                {
                    if (rhs[s.x][s.y] == moveCost(s, position) + g_old && !s.equals(goal))
                    {
                        rhs[s.x][s.y] = Float.POSITIVE_INFINITY;
                        for (Vec2 succ : getNeighbours(s))
                            rhs[s.x][s.y] = Math.min(rhs[s.x][s.y], moveCost(s, succ) + g[succ.x][succ.y]);
                    }
                    updateVertex(s);
                }
            }
        }

    }


    public List<Vec2> extractPath()
    {
        List<Vec2> path = new ArrayList<>();
        Set<Vec2> visited = new HashSet<>();
        path.add(current);
        visited.add(current);
        Vec2 cNode = current;

        while (!cNode.equals(goal))
        {
            Vec2 minNode = null;
            float minCost = Float.POSITIVE_INFINITY;
            for (Vec2 next : getNeighbours(cNode)){
                System.out.println(g[next.x][next.y] + " - " + minCost);
                if (g[next.x][next.y] <= minCost && !visited.contains(next)){
                    minCost = g[next.x][next.y];
                    minNode = next;
                }
            }
            System.out.println(minNode.repr());
            path.add(minNode);
            visited.add(minNode);
            cNode = minNode;
        }

        return path;
    }



    /*
    Get the neighbours of a position by excluding the obstacles.
    NOTE: The positions occupied by obstacles are not returned.
    */
    public List<Vec2> getNeighbours (Vec2 position)
    {
        List<Vec2> neighbours = new ArrayList<>(8);

        for (int i = position.x - 1; i < position.x + 2; i++)
        {
            for (int j = position.y - 1; j < position.y + 2; j++)
            {

                if (
                      (i != position.x || j != position.y) &&       // Exclude the exact position
                      i >= 0 &&                                     // Check we are inside the grid
                      i < slam.length &&                            // Check we are inside the grid
                      j >= 0 &&                                     // Check we are inside the grid
                      j < slam[0].length &&                         // Check we are inside the grid
                      slam[i][j] == 0                               // Avoid positions occupied by obstacles
                )
                    neighbours.add(new Vec2(i, j));

            }
        }

        return neighbours;
    }


    /*
   Get the neighbours of a position by including the obstacles.
   NOTE: The positions occupied by obstacles are not returned.
   */
    public List<Vec2> getNeighboursWithObstacles (Vec2 position)
    {
        List<Vec2> neighbours = new ArrayList<>(8);

        for (int i = position.x - 1; i < position.x + 2; i++)
        {
            for (int j = position.y - 1; j < position.y + 2; j++)
            {

                if (
                        (i != position.x || j != position.y) &&               // Exclude the exact position
                                i >= 0 &&                                     // Check we are inside the grid
                                i < slam.length &&                            // Check we are inside the grid
                                j >= 0 &&                                     // Check we are inside the grid
                                j < slam[0].length                            // Check we are inside the grid
                )
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
