package sx.gridmerger;


import org.apache.commons.lang3.tuple.ImmutablePair;
import org.apache.commons.lang3.tuple.Pair;
import sx.Vec2i;

import java.text.DateFormat;
import java.text.SimpleDateFormat;
import java.util.Date;
import java.util.LinkedList;


public class StochasticGridMerger implements GridMerger
{
    /*
        An instance of this class represents the implementation of the algorithm
        presented in Birk, A., & Carpin, S. (2006). Merging occupancy grid maps from
        multiple robots. Proceedings of the IEEE, 94(7), 1384-1397.

        The grid maps merging is computed through an iterative random path walk
        exploration.

        NOTE: In this class, the same assumptions made in the rest of the repository
        are made. Hence, the value of grid maps cells will be 0 for indicating a free
        space, 1 for indicating an obstacle, and -1 for indicating an unknown position.
    */


    public int steps;                       // The number of iterations computed
    public int memorySize;                  // The number of configurations considered in mean and covariance calculation
    public float mu, cov;                   // Mean and covariance used to generate a new sample

    public LinkedList<Pair<Transform,Float>> memory;    // The last configurations considered in mean and covariance calculation

    public Transform current;                // The currently considered transform
    public Transform best;                   // The best transform found so far
    public float currentDist;                // The distance between the reference matrix and the transformed matrix
                                             // after applying the current transform
    public float bestDist;                   // The distance between the reference matrix and the transformed matrix
                                             // after applying the best transform

    public float acceptanceFactor;           // If the acceptance is under this threshold, there is not enough overlap
                                             // between grid maps to compute a global grid map.
    public float cLock;                      // Scaling factor regulating the convergence speed


    public int[][] grid;                     // The reference grid map --i.e., the one usually kept as it is

    public int[][] dmap_free_ref;            // The distances map relative to free positions in reference grid map
    public int[][] dmap_obstacle_ref;        // The distances map relative to obstacles in reference grid map

    public int agreement, disagreement;     // Lastly computed agreement and disagreement


    // Time format used for logging
    private DateFormat timef = new SimpleDateFormat("HH:mm:ss.SSS");



    public StochasticGridMerger()
    {

    }


    public StochasticGridMerger (int steps, int memorySize, float mu_init, float cov_init, float acceptanceFactor, float cLock)
    {
        this.steps = steps;
        this.memorySize = memorySize;
        this.mu = mu_init;
        this.cov = cov_init;

        this.cLock = cLock;
        this.acceptanceFactor = acceptanceFactor;

        this.agreement = 0;
        this.disagreement = 0;

        this.memory = new LinkedList<>();
        this.current = null;
        this.best = null;
        this.currentDist = Float.POSITIVE_INFINITY;
        this.bestDist = Float.POSITIVE_INFINITY;
    }


    /* Set or update the reference grid map */
    public void setReferenceGrid (int[][] grid)
    {
        // Set the reference grid map
        this.grid = grid;

        //System.out.println("[" + timef.format(new Date()) + "][INFO] Computing distance maps");
        int X = grid.length, Y = grid[0].length;

        // Initialization of relative distance map matrices
        this.dmap_free_ref = new int[X][Y];
        this.dmap_obstacle_ref = new int[X][Y];

        // The maximum value the distance maps can assume
        int LARGE_NUMBER = X * Y + 1;

        // Starting values
        // dmap[i][j] is set to 0 if the value of the grid map is equal to the
        // currently explored, otherwise it is set to Infinite (i.e., a sufficiently big number)
        for (int x = 0; x < X; x++)
        {
            for (int y = 0; y < Y; y++)
            {
                dmap_free_ref[x][y] = (grid[x][y] == 0) ? 0 : LARGE_NUMBER;
                dmap_obstacle_ref[x][y] = (grid[x][y] == 1) ? 0 : LARGE_NUMBER;
            }
        }

        // First relaxation step for dstance map matrices
        for (int x = 1; x < X; x++)
        {
            for (int y = 1; y < Y; y++)
            {
                dmap_free_ref[x][y] = Math.min(dmap_free_ref[x][y], Math.min(dmap_free_ref[x - 1][y] + 1, dmap_free_ref[x][y - 1] + 1));
                dmap_obstacle_ref[x][y] = Math.min(dmap_obstacle_ref[x][y], Math.min(dmap_obstacle_ref[x - 1][y] + 1, dmap_obstacle_ref[x][y - 1] + 1));
            }
        }

        // Second relaxation step for distance map matrices
        for (int x = X - 2; x < 0; x--)
        {
            for (int y = Y - 2; y < 0; y--)
            {
                dmap_free_ref[x][y] = Math.min(dmap_free_ref[x][y], Math.min(dmap_free_ref[x + 1][y] + 1, dmap_free_ref[x][y + 1] + 1));
                dmap_obstacle_ref[x][y] = Math.min(dmap_obstacle_ref[x][y], Math.min(dmap_obstacle_ref[x + 1][y] + 1, dmap_obstacle_ref[x][y + 1] + 1));
            }
        }
    }


    /* Compute the acceptance indicator according to current agreement and disagreement */
    public float acceptanceIndicator ()
    {
        if (agreement + disagreement == 0)
            return 0.0f;
        return 1.0f - (agreement / (agreement + disagreement));
    }


    /* It defines if there is enough overlap to consider the merging acceptable */
    public boolean isAccepted ()
    {
        return acceptanceIndicator() > acceptanceFactor;
    }


    /* Compute the distance between the reference grid map and the transformed grid map passed as argument */
    public float computeDistance (int[][] m)
    {
        //System.out.println("[" + timef.format(new Date()) + "][INFO] Computing similarities");
        int X = grid.length, Y = grid[0].length;

        // Initialization of distances map matrices
        int[][] dmap_free = new int[X][Y];
        int[][] dmap_obstacle = new int[X][Y];

        // The maximum value the distance maps can assume
        int LARGE_NUMBER = X * Y + 1;

        // Starting values
        // dmap[i][j] is set to 0 if the value of the grid map is equal to the
        // currently explored, otherwise it is set to Infinite
        for (int x = 0; x < X; x++)
        {
            for (int y = 0; y < Y; y++)
            {
                dmap_free[x][y] = (m[x][y] == 0) ? 0 : LARGE_NUMBER;
                dmap_obstacle[x][y] = (m[x][y] == 1) ? 0 : LARGE_NUMBER;
            }
        }

        // First relaxation step for all distance map matrices
        for (int x = 1; x < X; x++)
        {
            for (int y = 1; y < Y; y++)
            {
                dmap_free[x][y] = Math.min(dmap_free[x][y], Math.min(dmap_free[x - 1][y] + 1, dmap_free[x][y - 1] + 1));
                dmap_obstacle[x][y] = Math.min(dmap_obstacle[x][y], Math.min(dmap_obstacle[x - 1][y] + 1, dmap_obstacle[x][y - 1] + 1));
            }
        }

        // Second relaxation step for all distance map matrices
        for (int x = X - 2; x < 0; x--)
        {
            for (int y = Y - 2; y < 0; y--)
            {
                dmap_free[x][y] = Math.min(dmap_free[x][y], Math.min(dmap_free[x + 1][y] + 1, dmap_free[x][y + 1] + 1));
                dmap_obstacle[x][y] = Math.min(dmap_obstacle[x][y], Math.min(dmap_obstacle[x + 1][y] + 1, dmap_obstacle[x][y + 1] + 1));
            }
        }

        // Init similarities and counters
        int d_m1_m2_free = 0, d_m2_m1_free = 0;
        int d_m1_m2_obstacle = 0, d_m2_m1_obstacle = 0;
        int N_free_m1 = 0, N_free_m2 = 0;
        int N_obstacle_m1 = 0, N_obstacle_m2 = 0;

        // Reset agreeemnt and disagreement
        agreement = 0; disagreement = 0;

        for (int x = 0; x < X; x++)
        {
            for (int y = 0; y < Y; y++)
            {
                // Update distance components
                if (grid[x][y] == 0)
                {
                    d_m1_m2_free += dmap_free[x][y];
                    N_free_m1++;
                }

                if (grid[x][y] == 1)
                {
                    d_m1_m2_obstacle += dmap_obstacle[x][y];
                    N_obstacle_m1++;
                }

                if (m[x][y] == 0)
                {
                    d_m2_m1_free += dmap_free_ref[x][y];
                    N_free_m2++;
                }

                if (m[x][y] == 1)
                {
                    d_m2_m1_obstacle += dmap_obstacle_ref[x][y];
                    N_obstacle_m2++;
                }

                // Update agreeement and disagreement
                if (grid[x][y] != -1 && m[x][y] != -1 && grid[x][y] == m[x][y])
                    agreement++;
                if (grid[x][y] != -1 && m[x][y] != -1 && grid[x][y] != m[x][y])
                    disagreement++;
            }
        }

        // Compute the image registration distance factor
        float f1 = (N_free_m1 > 0) ? d_m1_m2_free / N_free_m1 : 0.0f;
        float f2 = (N_obstacle_m1 > 0) ? d_m1_m2_obstacle / N_obstacle_m1 : 0.0f;
        float f3 = (N_free_m2 > 0) ? d_m2_m1_free / N_free_m2 : 0.0f;
        float f4 = (N_obstacle_m2 > 0) ? d_m2_m1_obstacle / N_obstacle_m2 : 0.0f;

        float fi = f1 + f2 + f3 + f4;

        // Compute the overlap factor
        float overlap = disagreement - agreement;

        // Return the distance
        return fi + cLock * overlap;
    }


    /* Method for caching a new transform in memory (only the best memorySize are kept) */
    /*
    public void remember (Transform transform, float distance)
    {
        if (memory.size() < memorySize)
            memory.add(new ImmutablePair<>(transform, distance));
        else
        {
            for (Pair solution : memory)
            {
                if ( (float) solution.getRight() > distance )
            }
        }
    }
    */


    /*
      Initialization with an iterative almost-exahaustive approach.

        :param minT: Lowest bound translation
        :param maxT: Highest bound translation
        :param step: Translation step
        :param angle: Rotation step
    */
    public void init (int[][] transformedGrid, int minT, int maxT, int step, float angle)
    {
        // Set rotation center in the middle of reference grid map
        Vec2i center = new Vec2i((int) (grid.length / 2.0f), (int) (grid[0].length / 2.0f));

        // Test all defined combinations
        for (int translationX = minT; translationX < maxT; translationX += step)

            for (int translationY = minT; translationY < maxT; translationY += step)

                for (float rotation = 0.0f; rotation < (float) Math.PI * 2.0f; rotation += angle)
                {

                    Transform transform = new Transform(translationX, translationY, center, rotation);
                    int[][] grid = GridMerger.transformGrid(transformedGrid, transform);
                    float distance = computeDistance(grid);

                }

    }


}
