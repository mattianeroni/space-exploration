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
    public int[][] updatedSlam;                // A more updated version of the slam with new obstacles or spaces
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
        this.updatedSlam = new int[slam.length][slam[0].length];
        for (int i = 0; i < slam.length; i++)
        {
            Arrays.fill(g[i], Float.POSITIVE_INFINITY);
            Arrays.fill(rhs[i], Float.POSITIVE_INFINITY);
            System.arraycopy( slam[i], 0, updatedSlam[i], 0, slam[i].length );
        }

        // Init heap and matrices
        rhs[goal.x][goal.y] = 0.0f;
        HeapNode hn = computeHeapNode(goal);
        inconsistents.put(goal, hn);
        heap.add(hn);

    }


    // The cost of moving from a position to a close position
    public static float moveCost(Vec2 p1, Vec2 p2, int[][] slam)
    {
        // Same position
        if (p1.equals(p2))
            return 0.0f;

        // Move into obstacle or from obstacle
        if (slam[p1.x][p1.y] == 1 || slam[p2.x][p2.y] == 1 )
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
        if (rhs[current.x][current.y] == Float.POSITIVE_INFINITY)
        {
            System.out.println("[WARNING] No known path to goal.");
            return;
        }


        // Move to the next position
        Vec2 nextNode = null;
        float minCost = Float.POSITIVE_INFINITY;
        for (Vec2 node : getNeighbours(current))
        {
            float cost = moveCost(current, node, slam) + g[node.x][node.y];
            if (cost <= minCost)
            {
                minCost = cost;
                nextNode = node;
            }
        }
        if (current == null)
            System.out.println("[WARNING] No possible next step.");

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


    /*
       Suggested function to update the updatedSlam (i.e., a new vision of the
       environment the algorithm is not considering yet).
        @param x: The x-coordinate
        @param y: The y-coordinate
        @param value: The new value (i.e., 1 if a new obstacle is detected, 0 if new passage is detected)
    */
    public void setUpdatedSlam (int x, int y, int value) {
        changed = true;
        updatedSlam[x][y] = value;
    }


    public void updateCosts()
    {
        // If no changed detected there is no need to update g and rhs values
        if (!changed)
            return;

        changed = false;
        km += moveCost(last, current, updatedSlam);
        last = current;

        for (int x = 0; x < slam.length; x++)
        {
            for (int y = 0; y < slam[0].length; y++)
            {
                // No change detected in current cell
                if (slam[x][y] == updatedSlam[x][y])
                    continue;

                Vec2 changedNode = new Vec2(x, y);

                // Iterate the cell's neighbours
                for (Vec2 neighbour : getNeighbours(changedNode))
                {
                    // Update neighbours from changed cell to neighbours
                    Vec2 u = changedNode, v = neighbour;
                    float c_old = moveCost(u, v, slam);
                    float c_new = moveCost(u, v, updatedSlam);

                    if (c_old > c_new && !u.equals(goal))
                    {
                        rhs[u.x][u.y] = Math.min(rhs[u.x][u.y], c_new + g[v.x][v.y]);
                    }
                    else if (rhs[u.x][u.y] == c_old + g[v.x][v.y] && !u.equals(goal))
                    {
                        rhs[u.x][u.y] = Float.POSITIVE_INFINITY;
                        for (Vec2 succ : getNeighbours(u))
                            rhs[u.x][u.y] = Math.min(rhs[u.x][u.y], moveCost(u, succ, updatedSlam) + g[succ.x][succ.y]);
                    }
                    updateVertex(u);

                    // Update edges from neighbours to changed cell
                    u = neighbour; v = changedNode;
                    c_old = moveCost(u, v, slam);
                    c_new = moveCost(u, v, updatedSlam);
                    if (c_old > c_new && !u.equals(goal))
                    {
                        rhs[u.x][u.y] = Math.min(rhs[u.x][u.y], c_new + g[v.x][v.y]);
                    }
                    else if (rhs[u.x][u.y] == c_old + g[v.x][v.y] && !u.equals(goal))
                    {
                        rhs[u.x][u.y] = Float.POSITIVE_INFINITY;
                        for (Vec2 succ : getNeighbours(u))
                            rhs[u.x][u.y] = Math.min(rhs[u.x][u.y], moveCost(u, succ, updatedSlam) + g[succ.x][succ.y]);
                    }
                    updateVertex(u);

                }
            }
        }
        // Update slam
        slam = Arrays.stream(updatedSlam).map(int[]::clone).toArray(int[][]::new);
    }


    // Compute the shortest path to the goal according to the current knowledge of the environment
    public void computePath()
    {


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
                        rhs[s.x][s.y] = Math.min( rhs[s.x][s.y], moveCost(s, position, slam) + g[position.x][position.y] );
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
                    if (rhs[s.x][s.y] == moveCost(s, position, slam) + g_old && !s.equals(goal))
                    {
                        rhs[s.x][s.y] = Float.POSITIVE_INFINITY;
                        for (Vec2 succ : getNeighbours(s))
                            rhs[s.x][s.y] = Math.min(rhs[s.x][s.y], moveCost(s, succ, slam) + g[succ.x][succ.y]);
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
            for (Vec2 next : getNeighbours(cNode))
            {
                float cost = g[next.x][next.y] + moveCost(cNode, next, slam);
                if (cost <= minCost && !visited.contains(next))
                {
                    minCost = cost;
                    minNode = next;

                }
            }
            if (cNode == null)
                System.out.println("[WARNING] No possible next step.");
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


    // The euclidean distance between two positions
    public static float euclidean (Vec2 v1, Vec2 v2)
    {
        return (float) Math.sqrt(Math.pow(v1.x - v2.x, 2) + Math.pow(v1.y - v2.y, 2));
    }


}
