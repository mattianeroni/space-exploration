package sx.pathfind;

import sx.Vec2;
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
    public Vec2 source, goal;                       // Starting and ending positions
    public Vec2 current;                            // The current position where the robot
    public Vec2 expandingNode;                      // The node the algorithm is currently expanding

    public Set<Vec2> covered;                       // The set of positions covered by the robot
    public LinkedList<Vec2> path;                   // The current minimum path to the goal
    public int[][] slam;                            // The current knowledge of the environment the algorithm has

    public Map<Vec2, Vec2> explored;                // HashMap of explored nodes to parent closest to the source

    // Logger time stamp format
    public SimpleDateFormat time = new SimpleDateFormat("HH:mm:ss.SSS");


    public AStar (Vec2 source, Vec2 goal, int[][] slam)
    {
        this.source = source;
        this.goal = goal;
        this.current = source;
        this.expandingNode = source;
        this.slam = slam;

        this.path = new LinkedList<>();
        this.heap = new PriorityQueue<>();
        this.explored = new HashMap<>();
        heap.add(new AStarHeapNode(0.0f, 0, current));
        counter = 0;
    }


    /* The cost of moving from a position to a close position */
    public float moveCost(Vec2 p1, Vec2 p2)
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


    /* Compute the heap node and the priority associated with a position */
    public AStarHeapNode computeHeapNode(Vec2 position)
    {
        return new AStarHeapNode(
            moveCost(current, position) + euclidean(position, goal),
                ++counter,
                position
        );
    }

    /*
        This method reset the computed path after a change in the environment
        and prepare the algorithm to a further path computation.
    */
    public void reset ()
    {
        expandingNode = current;
        path = new LinkedList<>();
        heap = new PriorityQueue<>();
        explored = new HashMap<>();
        heap.add(new AStarHeapNode(0.0f, 0, current));
        counter = 0;
    }


    /* Inform the algorithm that the robot is doing a step to the goal */
    @Override
    public void step()
    {
        // Arrived
        if (current.equals(goal))
            return;
        // Next step
        current = path.removeFirst();
        covered.add(current);
    }


    /*
        Backward reconstruction of minimum path from current robot
        position to goal position.
    */
    @Override
    public void extractPath() throws NoPathFound
    {
        /*
        path = [curnode]
        node = parent
        while node is not None:
            path.append(node)
            node = explored[node]
        path.reverse()
         */
        path = new LinkedList<>();


        Collections.reverse(path);

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
        /*
        # Maps enqueued nodes to distance of discovered paths and the
        # computed heuristics to target. We avoid computing the heuristics
        # more than once and inserting the node into the queue too many times.
        enqueued = {}
        # Maps explored nodes to parent closest to the source.
        explored = {}

         */
        // Start expanding the current robot position
        expandingNode = current;

        while (heap.peek() != null && !expandingNode.equals(goal))
        {

        }
    }


    /*
        Method used to update the SLAM (i.e., the view the robot has
        of the environment).
     */
    @Override
    public void updateSlam(Vec2 position, int value)
    {
        slam[position.x][position.y] = value;
    }


    /* Return the robot current position known by the algorithm */
    @Override
    public Vec2 getCurrent() {
        return current;
    }


    /* Method to return the currently considered minimum path */
    @Override
    public LinkedList<Vec2> getPath() {
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


    /* The euclidean distance between two positions */
    public static float euclidean (Vec2 v1, Vec2 v2)
    {
        if (v1.equals(v2))
            return 0.0f;
        return (float) Math.sqrt(Math.pow(v1.x - v2.x, 2) + Math.pow(v1.y - v2.y, 2));
    }

}
