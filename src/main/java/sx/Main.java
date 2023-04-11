package sx;

import org.locationtech.jts.geom.Envelope;
import org.locationtech.jts.index.SpatialIndex;
import org.locationtech.jts.index.quadtree.Quadtree;
import org.locationtech.jts.index.strtree.STRtree;
import sx.frontier.Frontier;
import sx.frontier.FrontierDetectorTester;
import sx.frontier.WavefrontFrontierDetector;
import sx.pathfind.*;
import processing.core.PApplet;
import sx.pathsmoother.GeometricSmoother;
import sx.pathsmoother.GradientAscent;

import java.util.*;

import static sx.GridMapReader.makeBinaryGridMap;


public class Main
{

    // The size of the grid map
    static int ROWS = 100;
    static int COLS = 50;


    public static void main (String[] args)
    {

        // Create the grid map
        int[][] grid = new int[ROWS][COLS];
        int[][] slam = new int[ROWS][COLS];

        for (int i = 0; i < grid.length; i++) {
            for (int j = 0; j < grid[0].length; j++) {
                if ( (i < 10 || i >= 20) || (j < 10 || j >= 20) ) {
                    grid[i][j] = -1;
                    slam[i][j] = -1;
                }
            }
        }


        WavefrontFrontierDetector detector = new WavefrontFrontierDetector(new Vec2i(15, 15), slam);

        FrontierDetectorTester tester = new FrontierDetectorTester(detector, grid, 15);
        PApplet.runSketch(new String[]{"ProcessingTest"}, tester);

    }



    public static void testPathFinding ()
    {
        // Create the grid map
        int[][] grid = new int[ROWS][COLS];

        // Create the SLAM (i.e., a partially blind view of the grid)
        int[][] slam = new int[ROWS][COLS];

        // Translate the grid map into a binary obstacles map
        //grid = makeBinaryGridMap(grid, 4);

        // Source and target points
        Vec2i source = new Vec2i(0, 0);
        Vec2i goal = new Vec2i(1999, 1499);

        // Initialise the path smoother
        //GradientAscent smoother = new GradientAscent(0.9f, 0.1f);
        GeometricSmoother smoother = new GeometricSmoother(30.0f, 2.0f);

        // Test D* Lite algorithm
        // Init the path finding algorithm
        DStar dstar = new DStar(source, goal, slam);
        // Init the game
        PathFinderTester tester = new PathFinderTester(dstar, smoother, grid, 50);
        PApplet.runSketch(new String[]{"ProcessingTest"}, tester);


        // Test A* algorithm
        // Init the path finding algorithm
        AStar astar = new AStar(source, goal, slam);

        // Init the game
        //PathFinderTester tester = new PathFinderTester(astar, grid, 2);
        //PApplet.runSketch(new String[]{"ProcessingTest"}, tester);
    }


}