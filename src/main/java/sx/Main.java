package sx;

import org.locationtech.jts.geom.Envelope;
import org.locationtech.jts.index.SpatialIndex;
import org.locationtech.jts.index.quadtree.Quadtree;
import org.locationtech.jts.index.strtree.STRtree;
import org.locationtech.jts.triangulate.quadedge.QuadEdgeSubdivision;
import sx.frontier.Frontier;
import sx.frontier.FrontierDetectorTester;
import sx.frontier.WavefrontFrontierDetector;
import sx.gridmerger.GridMerger;
import sx.gridmerger.StochasticGridMerger;
import sx.gridmerger.Transform;
import sx.pathfind.*;
import processing.core.PApplet;
import sx.pathsmoother.GeometricSmoother;
import sx.pathsmoother.GradientAscent;

import java.util.*;



public class Main
{

    /*
        NOTE: All this program assumes the following standard for grid maps:
            - 0: free space
            - -1: unknown space
            - 1: obstacle
    */

    public static void main (String[] args)
    {

        //testPathFinding();
        //testFrontierDetection();

        // The size of the grid map
        int ROWS = 10;
        int COLS = 10;

        // Create the grid map
        int[][] grid = {
                { -1, -1, -1, -1, -1, -1, -1, -1,  0,  0},
                { -1, -1, -1, -1, -1, -1, -1, -1,  0,  0},
                { -1,  0,  0,  0,  0,  0,  1,  1,  0,  0},
                { -1,  0,  0,  0,  0,  0,  0,  1,  0,  0},
                { -1,  0,  0,  0,  1,  0,  0,  0,  0,  0},
                { -1,  0,  0,  1,  1,  1,  0,  0,  0,  0},
                { -1,  0,  0,  0,  1,  0,  0,  0,  0,  0},
                { -1,  0,  0,  0,  0,  0,  0,  0,  0,  0},
                { -1,  0,  0,  0,  0,  0,  0,  0,  0,  0},
                { -1,  0,  0,  0,  0,  0,  0,  0,  0,  0},
        };


        //Vec2i source = new Vec2i(10, 10);
        Vec2i center = new Vec2i(5, 5);
        float angle = (float) Math.PI / 2.0f;
        Vec2i translation = new Vec2i(-2, 0);

        int[][] rotGrid = GridMerger.rotateGrid(grid, center, angle);
        int[][] tranGrid = GridMerger.translateGrid(grid, translation);
        int[][] newGrid = GridMerger.translateGrid( GridMerger.rotateGrid(grid, center, angle ), translation);
        int[][] nGrid = GridMerger.transformGrid(grid, new Transform(translation,  center, angle));

        plotGrid(grid);
        System.out.println("-----------------------------------");
        plotGrid(nGrid);
        System.out.println("-----------------------------------");
        plotGrid(newGrid);
    }


    /* Method to make a beauty plot of a grid map */
    public static void plotGrid (int[][] grid)
    {
        for (int i = 0; i < grid.length; i++)
        {
            for (int j = 0; j < grid[0].length; j++)
            {
                int val = grid[i][j];
                String prefix = (val == 0 || val == 1) ? " " : "";
                System.out.print(prefix + val + ", ");
            }
            System.out.print("\n");
        }
    }


    public static void testFrontierDetection ()
    {
        // The size of the grid map
        int ROWS = 100;
        int COLS = 50;

        // Create the grid map
        int[][] grid = new int[ROWS][COLS];
        int[][] slam = new int[ROWS][COLS];

        // Init already explored area
        for (int i = 0; i < grid.length; i++)
            for (int j = 0; j < grid[0].length; j++)
                if ( ( i >= 10) || ( j >= 10) )
                {
                    grid[i][j] = -1;
                    slam[i][j] = -1;
                }

        WavefrontFrontierDetector detector = new WavefrontFrontierDetector(new Vec2i(5, 5), slam);

        FrontierDetectorTester tester = new FrontierDetectorTester(detector, grid, 10);
        PApplet.runSketch(new String[]{"ProcessingTest"}, tester);
    }



    public static void testPathFinding ()
    {
        // The size of the grid map
        int ROWS = 80;
        int COLS = 40;

        // Create the grid map
        int[][] grid = new int[ROWS][COLS];

        // Create the SLAM (i.e., a partially blind view of the grid)
        int[][] slam = new int[ROWS][COLS];


        // Source and target points
        Vec2i source = new Vec2i(0, 0);
        Vec2i goal = new Vec2i(79, 39);

        // Initialise the path smoother
        //GradientAscent smoother = new GradientAscent(0.9f, 0.1f);
        GeometricSmoother smoother = new GeometricSmoother(30.0f, 2.0f);

        // Test D* Lite algorithm
        // Init the path finding algorithm
        DStar dstar = new DStar(source, goal, slam);
        // Init the game
        PathFinderTester tester = new PathFinderTester(dstar, smoother, grid, 30);
        PApplet.runSketch(new String[]{"ProcessingTest"}, tester);


        // Test A* algorithm
        // Init the path finding algorithm
        //AStar astar = new AStar(source, goal, slam);

        // Init the game
        //PathFinderTester tester = new PathFinderTester(astar, smoother, grid, 30);
        //PApplet.runSketch(new String[]{"ProcessingTest"}, tester);
    }


}