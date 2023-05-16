package sx.gridmerger;


import sx.Vec2i;

import java.text.DateFormat;
import java.text.SimpleDateFormat;


public class StochasticGridMerger
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


    public int steps;                        // The number of iterations computed
    public int memorySize;                   // The number of configurations considered in mean and covariance calculation
    public float mu, cov;                    // Mean and covariance used to generate a new sample

    public LimitedSizeSet<Solution> memory;  // The last configurations considered in mean and covariance calculation

    public Solution current;                 // The currently considered transform
    public Solution best;                    // The best transform found so far

    public float acceptanceFactor;           // If the acceptance is under this threshold, there is not enough overlap
                                             // between grid maps to compute a global grid map.
    public float cLock;                      // Scaling factor regulating the convergence speed


    public int[][] grid;                     // The reference grid map --i.e., the one usually kept as it is

    public int[][] dmap_free_ref;            // The distances map relative to free positions in reference grid map (computed only once)
    public int[][] dmap_obstacle_ref;        // The distances map relative to obstacles in reference grid map (computed only once)



    // Time format used for logging
    private DateFormat timef = new SimpleDateFormat("HH:mm:ss.SSS");



    public StochasticGridMerger (int memorySize, float acceptanceFactor, float cLock, int steps)
    {
        this.steps = steps;
        this.memorySize = memorySize;
        this.memory = new LimitedSizeSet<>(memorySize);

        this.cLock = cLock;
        this.acceptanceFactor = acceptanceFactor;

        this.current = null;
        this.best = null;
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


    /* It defines if there is enough overlap to consider a solution acceptable */
    public boolean isAccepted (Solution solution)
    {
        return solution.acceptanceIndicator() > acceptanceFactor;
    }


    /*
        Compute the distance between the reference grid map and the transformed grid map (i.e., secondGrid),
        after applying the transform passed as argument.
        :param secondGrid:
        :param transform:
    */
    public Solution computeTransform (int[][] secondGrid, Transform transform)
    {
        int X = grid.length, Y = grid[0].length;

        // Apply the transform
        int[][] transformedGrid = GridTransformer.transformGrid(secondGrid, transform);

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
                dmap_free[x][y] = (transformedGrid[x][y] == 0) ? 0 : LARGE_NUMBER;
                dmap_obstacle[x][y] = (transformedGrid[x][y] == 1) ? 0 : LARGE_NUMBER;
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
        int agreement = 0, disagreement = 0;

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

                if (transformedGrid[x][y] == 0)
                {
                    d_m2_m1_free += dmap_free_ref[x][y];
                    N_free_m2++;
                }

                if (transformedGrid[x][y] == 1)
                {
                    d_m2_m1_obstacle += dmap_obstacle_ref[x][y];
                    N_obstacle_m2++;
                }

                // Update agreeement and disagreement
                if (grid[x][y] != -1 && transformedGrid[x][y] != -1 && grid[x][y] == transformedGrid[x][y])
                    agreement++;
                if (grid[x][y] != -1 && transformedGrid[x][y] != -1 && grid[x][y] != transformedGrid[x][y])
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
        return new Solution(transform, fi + cLock * overlap, agreement, disagreement);
    }


    /*
      Initialization with an iterative almost-exahaustive approach.
        :param minT: Lowest bound translation
        :param maxT: Highest bound translation
        :param step: Translation step
        :param angle: Rotation step
    */
    public void exhaustiveSearch (int[][] secondGrid, int minT, int maxT, int stepT, float angle)
    {
        // Set rotation center in the middle of reference grid map
        Vec2i center = new Vec2i((int) (grid.length / 2.0f), (int) (grid[0].length / 2.0f));

        // Initialise the current and best solutions
        this.current = computeTransform(secondGrid, new Transform(0, 0, center, 0.0f));
        this.best = current;

        // Initialise the memory of N best explored solutions
        this.memory = new LimitedSizeSet<>(memorySize);
        memory.add(best);

        // Test all defined combinations
        for (int translationX = minT; translationX < maxT; translationX += stepT)
        {
            for (int translationY = minT; translationY < maxT; translationY += stepT)
            {
                for (float rotation = 0.0f; rotation < (float) Math.PI * 2.0f; rotation += angle)
                {

                    // Measure the effect of the transform
                    Transform transform = new Transform(translationX, translationY, center, rotation);
                    Solution sol = computeTransform(secondGrid, transform);

                    System.out.println(transform.repr() + " - " + sol.acceptanceIndicator() + " - " + sol.distance + " - " + best.distance);

                    // Eventually update the memory and the current best solution found so far
                    memory.add(sol);
                    if (sol.distance < best.distance)
                    {
                        current = sol;
                        best = sol;
                    }

                }  // end rotations
            }   // end y-translations
        }   // end x-translations

    }


}
