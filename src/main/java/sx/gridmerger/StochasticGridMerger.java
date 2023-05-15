package sx.gridmerger;


import java.text.DateFormat;
import java.text.SimpleDateFormat;
import java.util.Date;


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


    public int steps;                   // The number of iterations computed
    public int memory;                  // The last configurations considered in mean and covariance calculation
    public float mu, cov;               // Mean and covariance used to generate a new sample


    public float acceptanceFactor;      // If the acceptance is under this threshold, there is not enough overlap
                                        // between grid maps to compute a global grid map.
    public float cLock;                 // Scaling factor regulating the convergence speed


    public int[][] m1;                  // The reference grid map --i.e., the one usually kept as it is
    public int[][] m2;                  // The transformed grid map --i.e., the one to be transformed to identify a
                                        // global grid map

    public int[][] dmap_free_m1;            // The distances map relative to free positions in reference grid map
    public int[][] dmap_free_m2;            // The distances map relative to free positions in transformed grid map
    public int[][] dmap_obstacle_m1;        // The distances map relative to obstacles in reference grid map
    public int[][] dmap_obstacle_m2;        // The distances map relative to obstacles in transformed grid map

    public int agreement, disagreement;     // Current agreement and disagreement

    // Time format used for logging
    private DateFormat timef = new SimpleDateFormat("HH:mm:ss.SSS");



    public StochasticGridMerger()
    {

    }


    public StochasticGridMerger (int steps, int memory, float mu_init, float cov_init, float acceptanceFactor, float cLock)
    {
        this.steps = steps;
        this.memory = memory;
        this.mu = mu_init;
        this.cov = cov_init;

        this.cLock = cLock;
        this.acceptanceFactor = acceptanceFactor;

        this.agreement = 0;
        this.disagreement = 0;
    }


    /* Set or update the reference grid map */
    public void setFirstGrid (int[][] grid)
    {
        // Set the reference grid map
        this.m1 = grid;

        //System.out.println("[" + timef.format(new Date()) + "][INFO] Computing distance maps");
        int X = grid.length, Y = grid[0].length;

        // Initialization of relative distance map matrices
        dmap_free_m1 = new int[X][Y];
        dmap_obstacle_m1 = new int[X][Y];

        // The maximum value the distance maps can assume
        int LARGE_NUMBER = X * Y + 1;

        // Starting values
        // dmap[i][j] is set to 0 if the value of the grid map is equal to the
        // currently explored, otherwise it is set to Infinite (i.e., a sufficiently big number)
        for (int x = 0; x < X; x++)
        {
            for (int y = 0; y < Y; y++)
            {
                dmap_free_m1[x][y] = (m1[x][y] == 0) ? 0 : LARGE_NUMBER;
                dmap_obstacle_m1[x][y] = (m1[x][y] == 1) ? 0 : LARGE_NUMBER;
            }
        }

        // First relaxation step for dstance map matrices
        for (int x = 1; x < X; x++)
        {
            for (int y = 1; y < Y; y++)
            {
                dmap_free_m1[x][y] = Math.min(dmap_free_m1[x][y], Math.min(dmap_free_m1[x - 1][y] + 1, dmap_free_m1[x][y - 1] + 1));
                dmap_obstacle_m1[x][y] = Math.min(dmap_obstacle_m1[x][y], Math.min(dmap_obstacle_m1[x - 1][y] + 1, dmap_obstacle_m1[x][y - 1] + 1));
            }
        }

        // Second relaxation step for distance map matrices
        for (int x = X - 2; x < 0; x--)
        {
            for (int y = Y - 2; y < 0; y--)
            {
                dmap_free_m1[x][y] = Math.min(dmap_free_m1[x][y], Math.min(dmap_free_m1[x + 1][y] + 1, dmap_free_m1[x][y + 1] + 1));
                dmap_obstacle_m1[x][y] = Math.min(dmap_obstacle_m1[x][y], Math.min(dmap_obstacle_m1[x + 1][y] + 1, dmap_obstacle_m1[x][y + 1] + 1));
            }
        }
    }


    /* Set or update the transformed grid map */
    public void setSecondGrid (int[][] grid)
    {
        // Set the transformed grid map
        this.m2 = grid;

        //System.out.println("[" + timef.format(new Date()) + "][INFO] Computing distance maps");
        int X = grid.length, Y = grid[0].length;

        // Initialization
        dmap_free_m2 = new int[X][Y];
        dmap_obstacle_m2 = new int[X][Y];

        // The maximum value the distance maps can assume
        int LARGE_NUMBER = X * Y + 1;

        // Starting values
        // dmap[i][j] is set to 0 if the value of the grid map is equal to the
        // currently explored, otherwise it is set to Infinite
        for (int x = 0; x < X; x++)
        {
            for (int y = 0; y < Y; y++)
            {
                dmap_free_m2[x][y] = (m2[x][y] == 0) ? 0 : LARGE_NUMBER;
                dmap_obstacle_m2[x][y] = (m2[x][y] == 1) ? 0 : LARGE_NUMBER;
            }
        }

        // First relaxation step for all distance map matrices
        for (int x = 1; x < X; x++)
        {
            for (int y = 1; y < Y; y++)
            {
                dmap_free_m2[x][y] = Math.min(dmap_free_m2[x][y], Math.min(dmap_free_m2[x - 1][y] + 1, dmap_free_m2[x][y - 1] + 1));
                dmap_obstacle_m2[x][y] = Math.min(dmap_obstacle_m2[x][y], Math.min(dmap_obstacle_m2[x - 1][y] + 1, dmap_obstacle_m2[x][y - 1] + 1));
            }
        }

        // Second relaxation step for all distance map matrices
        for (int x = X - 2; x < 0; x--)
        {
            for (int y = Y - 2; y < 0; y--)
            {
                dmap_free_m2[x][y] = Math.min(dmap_free_m2[x][y], Math.min(dmap_free_m2[x + 1][y] + 1, dmap_free_m2[x][y + 1] + 1));
                dmap_obstacle_m2[x][y] = Math.min(dmap_obstacle_m2[x][y], Math.min(dmap_obstacle_m2[x + 1][y] + 1, dmap_obstacle_m2[x][y + 1] + 1));
            }
        }
    }


    /* Compute the acceptance indicator according to current agreement and disagreement */
    public float acceptanceIndicator ()
    {
        return 1.0f - (agreement / (agreement + disagreement));
    }


    /* It defines if there is enough overlap to consider the merging acceptable */
    public boolean isAccepted ()
    {
        return acceptanceIndicator() >= acceptanceFactor;
    }



    public float computeDistance ()
    {
        //System.out.println("[" + timef.format(new Date()) + "][INFO] Computing similarities");
        int X = m1.length, Y = m1[0].length;

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
                if (m1[x][y] == 0)
                {
                    d_m1_m2_free += dmap_free_m2[x][y];
                    N_free_m1++;
                }

                if (m1[x][y] == 1)
                {
                    d_m1_m2_obstacle += dmap_obstacle_m2[x][y];
                    N_obstacle_m1++;
                }

                if (m2[x][y] == 0)
                {
                    d_m2_m1_free += dmap_free_m1[x][y];
                    N_free_m2++;
                }

                if (m2[x][y] == 1)
                {
                    d_m2_m1_obstacle += dmap_obstacle_m1[x][y];
                    N_obstacle_m2++;
                }

                // Update agreeement and disagreement
                if (m1[x][y] != -1 && m2[x][y] != -1 && m1[x][y] == m2[x][y])
                    agreement++;
                if (m1[x][y] != -1 && m2[x][y] != -1 && m1[x][y] != m2[x][y])
                    disagreement++;
            }
        }

        // Compute the image registration distance factor
        float fi = (d_m1_m2_free / N_free_m1 + d_m1_m2_obstacle / N_obstacle_m1)
                + (d_m2_m1_free / N_free_m2 + d_m2_m1_obstacle / N_obstacle_m2);

        // Compute the overlap factor
        float overlap = disagreement - agreement;

        // Return the distance
        return fi + cLock * overlap;
    }


}
