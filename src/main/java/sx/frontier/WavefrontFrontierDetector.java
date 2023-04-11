package sx.frontier;

import org.locationtech.jts.geom.Envelope;
import org.locationtech.jts.index.quadtree.Quadtree;
import sx.Vec2i;

import java.util.*;


public class WavefrontFrontierDetector implements FrontierDetector
{

    /*
        An instance of this class represents a customized implementation
        of Wavefront Frontier Detection algorithm.
        Which in enhanced using a Rtree (to consider possibility of QuadTree
        or STRtree) to a faster detection of the frontiers that should be extended.
    */

    public Vec2i robotPos;                  // The current robot position
    public int[][] slam;                    // The environment as seen by the algorithm
                                            // (i.e., -1 unknown, 0 free known, 1 obstacle)

    public List<Frontier> frontiers;        // The list of currently detected frontiers
    public Quadtree rtree;                  // The Rtree that keeps track of frontiers in space for a faster detection
    public Vec2i minBound, maxBound;        // The cells defining the bunding box of last visited positions
    public int[][] visited;                 // Binary matrix that sign already visited positions (0 not visited, 1 visited)
                                            // NOTE: We intend visited by the wavefront algorithm not by the robot.
    public int heapCounter;                 // A counter to ensure the order in the heap queue



    public WavefrontFrontierDetector (Vec2i robotPos, int[][] slam)
    {
        this.robotPos = robotPos;
        this.slam = slam;
        this.visited = new int[slam.length][slam[0].length];

        this.slam[robotPos.x][robotPos.y] = 0;
        //this.visited[robotPos.x][robotPos.y] = 1;

        this.minBound = new Vec2i(Integer.MAX_VALUE, Integer.MAX_VALUE);
        this.maxBound = new Vec2i(Integer.MIN_VALUE, Integer.MIN_VALUE);
        this.frontiers = new ArrayList<>();
        this.rtree = new Quadtree();            // Eventually consider the usage of different Rtree
    }


    public PriorityQueue<FrontierHeapNode> initExplorationQueue()
    {

        PriorityQueue<FrontierHeapNode> queue = new PriorityQueue<>();

        if (frontiers.size() == 0) {
            heapCounter = 1;
            queue.add(new FrontierHeapNode(0, robotPos));
            return queue;
        }

        heapCounter = 0;
        List<Frontier> toRecompute = rtree.query(new Envelope(minBound.x, maxBound.x, minBound.y, maxBound.y));
        for (Frontier f : toRecompute)
        {
            rtree.remove(f.box, f);
            frontiers.remove(f);
            for (Vec2i cell : f.cells)
            {
                visited[cell.x][cell.y] = 0;
                queue.add(new FrontierHeapNode(heapCounter++, cell));
            }
        }

        return queue;
    }


    @Override
    public void computeDetection()
    {
        PriorityQueue<FrontierHeapNode> queue = initExplorationQueue();

        while (queue.peek() != null)
        {
            // Get the next position to expand
            FrontierHeapNode expandingNode = queue.poll();
            Vec2i position = expandingNode.position;

            // Exit if the position has already been visited
            if (visited[position.x][position.y] == 1)
                continue;

            // It is marked as visited and expanded
            visited[position.x][position.y] = 1;

            // If the position is not a frontier it is expanded
            if (!isFrontier(position))
            {
                for (Vec2i i : getNeighbours(position))
                    if (visited[i.x][i.y] == 0)
                        queue.add(new FrontierHeapNode(heapCounter++, i));

                continue;
            }

            // If we found a frontier we start exploring it with a nested Breath First Search
            expandFrontierFrom(position);

        }
    }


    /* Method used to expand a frontier starting from one of its elements */
    public void expandFrontierFrom (Vec2i position)
    {
        // Init inner queue
        int innerCounter = 0;
        PriorityQueue<FrontierHeapNode> innerQueue = new PriorityQueue<>();
        innerQueue.add(new FrontierHeapNode(0, position));

        // Init the set of cells part of the forntier
        Set<Vec2i> frontierSet = new HashSet<>();

        // Expand the frontier
        while (innerQueue.peek() != null)
        {
            FrontierHeapNode node = innerQueue.poll();
            Vec2i cPos = node.position;

            frontierSet.add(cPos);

            for (Vec2i neighbour : getNeighbours(cPos))
                if (isFrontier(neighbour) && !frontierSet.contains(neighbour))
                {
                    visited[neighbour.x][neighbour.y] = 1;
                    innerQueue.add(new FrontierHeapNode(innerCounter++, neighbour));
                }

        }

        // Init the frontier
        Frontier frontier = new Frontier(frontierSet);
        frontiers.add(frontier);
        rtree.insert(frontier.box, frontier);

    }



    @Override
    public void reset()
    {
        this.visited = new int[slam.length][slam[0].length];
        this.slam[robotPos.x][robotPos.y] = 0;
        //this.visited[robotPos.x][robotPos.y] = 1;
        this.minBound = new Vec2i(Integer.MAX_VALUE, Integer.MAX_VALUE);
        this.maxBound = new Vec2i(Integer.MIN_VALUE, Integer.MIN_VALUE);
        this.frontiers = new ArrayList<>();
        this.rtree = new Quadtree();            // Eventually consider the usage of different Rtree
    }


    @Override
    public void updateSlam(int x, int y, int value)
    {
        // Update the slam
        slam[x][y] = value;

        // Expand the bounding box of visited positions
        if (value == 0)
        {
            minBound.x = Math.min(x, minBound.x);
            minBound.y = Math.min(y, minBound.y);
            maxBound.x = Math.max(x, maxBound.x);
            maxBound.y = Math.max(y, maxBound.y);
        }
    }


    @Override
    public List<Frontier> getFrontiers()
    {
        return frontiers;
    }


    @Override
    public Vec2i getRobotPosition()
    {
        return robotPos;
    }


    public boolean isFrontier (Vec2i position)
    {
        int x = position.x, y = position.y;

        // Only free positions can be frontiers
        if (slam[x][y] == -1 || slam[x][y] == 1)
            return false;

        // Surrounding positions (only vertically and horizontally)
        int[][] surr = {
            {x - 1, y},
            {x + 1, y},
            {x, y - 1},
            {x, y + 1}
        };

        // If at least a surrounding position is unknown, the position is a frontier
        for (int[] pos : surr)
        {
            if (pos[0] >= 0 &&  pos[0] < slam.length && pos[1] >= 0 && pos[1] < slam[0].length && slam[pos[0]][pos[1]] == -1)
                return true;
        }

        return false;
    }


    public List<Vec2i> getNeighbours (Vec2i position)
    {
        List<Vec2i> neighbours = new ArrayList<>(8);

        for (int i = position.x - 1; i < position.x + 2; i++)
        {
            for (int j = position.y - 1; j < position.y + 2; j++)
            {
                if (
                        (i != position.x || j != position.y) &&               // Exclude the exact position
                        i >= 0 &&  i < slam.length &&                         // Check we are inside the grid
                        j >= 0 && j < slam[0].length &&                       // Check we are inside the grid
                        slam[i][j] == 0  &&                                   // Avoid obstacles and unknown cells
                        visited[i][j] == 0                                    // Avoid already visited cells
                )
                    neighbours.add(new Vec2i(i, j));
            }
        }
        return neighbours;
    }

}
