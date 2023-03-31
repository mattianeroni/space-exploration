package sx.pathfind;

import sx.Vec2i;
import sx.pathfind.exceptions.NoPathFound;

import java.text.SimpleDateFormat;
import java.util.*;



public class AStar implements PathFinder
{

    /*
        An instance of this class represents an implementation of the A* algorithm
        presented in Hart, P., Nilsson, N., & Raphael, B. (1968). A Formal Basis for the
        Heuristic Determination of Minimum Cost Paths. IEEE Transactions on Systems Science
        and Cybernetics, 4(2), 100–107.
    */

    public PriorityQueue<AStarHeapNode> heap;       // The priority queue of open positions to explore
    public int counter;                             // Counter of positions added to the heap
    public Vec2i source, goal;                       // Starting and ending positions
    public Vec2i current;                            // The current position where the robot
    public Vec2i expandingNode;                      // The node the algorithm is currently expanding

    public Set<Vec2i> covered;                       // The set of positions covered by the robot
    public LinkedList<Vec2i> path;                   // The current minimum path to the goal
    public int[][] slam;                            // The current knowledge of the environment the algorithm has

    public Map<Vec2i, Vec2i> explored;                // HashMap of explored nodes to parent closest to the source
                                                    // (needed to reconstruct the path from goal to current robot position)
    public Map<Vec2i, AStarEstimation> enqueued;     // Maps enqueued nodes to distance of discovered paths and the
                                                    // computed heuristics to target. In this way, we avoid computing the heuristics
                                                    // more than once and inserting the node into the queue too many times.

    // Logger time stamp format
    public SimpleDateFormat time = new SimpleDateFormat("HH:mm:ss.SSS");


    public AStar (Vec2i source, Vec2i goal, int[][] slam)
    {
        this.source = source;
        this.goal = goal;
        this.current = source;
        this.expandingNode = source;
        this.slam = slam;

        this.path = new LinkedList<>();
        this.heap = new PriorityQueue<>();
        heap.add(new AStarHeapNode(0.0f, 0, current, 0.0f, null));
        counter = 0;

        this.explored = new HashMap<>();
        this.enqueued = new HashMap<>();
        //this.dists = new float[slam.length][slam[0].length];
    }


    /* The cost of moving from a position to a close position */
    public float moveCost(Vec2i p1, Vec2i p2)
    {
        // Same position
        if (p1.equals(p2))
            return 0.0f;

        // Move into obstacle or from obstacle
        if ( slam[p1.x][p1.y] == 1 || slam[p2.x][p2.y] == 1 )
            return Float.POSITIVE_INFINITY;

        // Vertical or horizontal movement
        if ( p1.x == p2.x || p1.y == p2.y )
            return 1.0f;

        // Diagonal movement
        return 1.414f;
    }


    /*
        Compute the heap node and the priority associated with a position.
        Just a wrapper in order not to manage the counter.
    */
    public AStarHeapNode computeHeapNode(float estimation, Vec2i position, float cost, Vec2i parent)
    {
        return new AStarHeapNode(estimation, ++counter, position, cost, parent);
    }


    /*
    Compute the estimation of a position to store in enqueue hashmap in order
    to avoid to compute the heuristic too many times.

    public AStarEstimation computeEstimation (Vec2 position)
    {
        return new AStarEstimation(
            position,
            dists[expandingNode.x][expandingNode.y] + moveCost(expandingNode, position),
            euclidean(position, goal),
            current,
            goal
        );
    }
    */



    /*
        This method reset the computed path after a change in the environment
        and prepare the algorithm to a further path computation.
    */
    public void reset ()
    {
        expandingNode = current;
        path = new LinkedList<>();
        heap = new PriorityQueue<>();
        heap.add(new AStarHeapNode(0.0f, 0, current, 0.0f, null));
        counter = 0;
        this.explored = new HashMap<>();
        this.enqueued = new HashMap<>();
        //this.dists = new float[slam.length][slam[0].length];
    }


    /* Inform the algorithm that the robot is doing a step to the goal */
    @Override
    public void step()
    {
        // Arrived
        if (current.equals(goal))
            return;
        // Next step
        path.removeFirst();
        current = path.getFirst();
        covered.add(current);
    }


    /*
        Backward reconstruction of minimum path from current robot
        position to goal position.
    */
    @Override
    public void extractPath() throws NoPathFound
    {
        path = new LinkedList<>();
        path.add(goal);
        Vec2i node = explored.get(goal);

        while (node != null)
        {
            path.add(node);
            node = explored.get(node);
        }
        Collections.reverse(path);

        // Path not found
        if (!path.getFirst().equals(current))
            throw new NoPathFound("[" + time.format(new Date()) + "][ERROR] No path found");
    }


    @Override
    public void computePath()
    {
        reset();
        computeStartingPath();
    }


    @Override
    public void computeStartingPath()
    {

        // Start expanding the current robot position
        expandingNode = current;

        while (heap.peek() != null && !expandingNode.equals(goal))
        {
            // Pop the next node to expand (at first iteration it's the current robot position)
            AStarHeapNode expandingHeapNode = heap.poll();

            // Extract information from the node
            expandingNode = expandingHeapNode.position;
            float dist = expandingHeapNode.cost;
            Vec2i parent = expandingHeapNode.parent;

            if (explored.containsKey(expandingNode))
            {
                // Do not override the parent of current robot position
                if (explored.get(expandingNode) == null)
                    continue;

                // Skip bad paths enqueued before finding a shorter one
                AStarEstimation estimation = enqueued.get(expandingNode);
                if (estimation.cost < dist)
                    continue;
            }

            // Set parent
            explored.put(expandingNode, parent);

            for (Vec2i neighbour : getNeighbours(expandingNode))
            {
                float ncost = dist + moveCost(expandingNode, neighbour);
                float heuristic;
                if (enqueued.containsKey(neighbour))
                {

                    AStarEstimation estimation = enqueued.get(expandingNode);
                    heuristic = estimation.h;

                    if (estimation.cost < ncost)
                        continue;

                } else
                {

                    heuristic = euclidean(neighbour, goal);

                }

                enqueued.put(neighbour, new AStarEstimation(neighbour, ncost, heuristic));
                heap.add(computeHeapNode(ncost + heuristic, neighbour, ncost, expandingNode));
            }

        }

        try {
            extractPath();
        } catch (NoPathFound ex) {
            throw new RuntimeException(ex);
        }
    }


    /*
        Method used to update the SLAM (i.e., the view the robot has
        of the environment).
    */
    @Override
    public void updateSlam(Vec2i position, int value)
    {
        slam[position.x][position.y] = value;
    }


    /* Return the robot current position known by the algorithm */
    @Override
    public Vec2i getCurrent()
    {
        return current;
    }


    /* Method to return the currently considered minimum path */
    @Override
    public LinkedList<Vec2i> getPath()
    {
        return path;
    }


    /*
      Get the neighbours of a position by excluding the obstacles.
      NOTE: The positions occupied by obstacles are not returned.
    */
    public List<Vec2i> getNeighbours (Vec2i position)
    {
        List<Vec2i> neighbours = new ArrayList<>(8);

        for (int i = position.x - 1; i < position.x + 2; i++)
        {
            for (int j = position.y - 1; j < position.y + 2; j++)
            {

                if (
                        (i != position.x || j != position.y) &&               // Exclude the exact position
                                i >= 0 &&                                     // Check we are inside the grid
                                i < slam.length &&                            // Check we are inside the grid
                                j >= 0 &&                                     // Check we are inside the grid
                                j < slam[0].length &&                         // Check we are inside the grid
                                slam[i][j] == 0                               // Avoid positions occupied by obstacles
                )
                    neighbours.add(new Vec2i(i, j));

            }
        }
        return neighbours;
    }


    /* The euclidean distance between two positions */
    public static float euclidean (Vec2i v1, Vec2i v2)
    {
        if (v1.equals(v2))
            return 0.0f;
        return (float) Math.sqrt(Math.pow(v1.x - v2.x, 2) + Math.pow(v1.y - v2.y, 2));
    }

}
