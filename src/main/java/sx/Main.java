package sx;

import sx.pathfind.*;
import processing.core.PApplet;
import sx.pathsmoother.GeometricSmoother;
import sx.pathsmoother.GradientAscent;

import java.util.Arrays;
import java.util.Collections;
import java.util.List;


public class Main
{

    // The size of the grid map
    static int ROWS = 20;
    static int COLS = 10;


    public static void main(String[] args)
    {

        // Create the grid map
        int[][] grid = new int[ROWS][COLS];

        // Create the SLAM (i.e., a partially blind view of the grid)
        int[][] slam = new int[ROWS][COLS];

        // Translate the grid map into a binary obstacles map
        // grid = makeBinaryGridMap(grid, BRAVERY);

        // Source and target points
        Vec2i source = new Vec2i(0, 0);
        Vec2i goal = new Vec2i(19, 9);

        // Initialise the path smoother
        //GradientAscent smoother = new GradientAscent(0.9f, 0.1f);
        GeometricSmoother smoother = new GeometricSmoother(30.0f, 0.1f);

        // Test D* Lite algorithm
        // Init the path finding algorithm
        //DStar dstar = new DStar(source, goal, slam);
        // Init the game
        //PathFinderTester tester = new PathFinderTester(dstar, smoother, grid, 50);
        //PApplet.runSketch(new String[]{"ProcessingTest"}, tester);


        // Test A* algorithm
        // Init the path finding algorithm
        AStar astar = new AStar(source, goal, slam);

        // Init the game
        PathFinderTester tester = new PathFinderTester(astar, smoother, grid, 50);
        PApplet.runSketch(new String[]{"ProcessingTest"}, tester);

        /*
        Vec2f a = new Vec2f(0.0f, 0.0f);
        Vec2f b = new Vec2f(-10.0f, -100.0f);
        Vec2f c = new Vec2f(0.0f, 5.0f);
        float radius = 10.0f;
        Vec2f p = getCircleLineIntersectionPoint(a, b, c, radius);

        if (p != null)
            System.out.println(p.repr());
        else
            System.out.println(p);

        */
    }


}