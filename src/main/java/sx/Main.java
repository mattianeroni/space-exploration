package sx;

import sx.pathfind.AStar;
import sx.pathfind.AStarHeapNode;
import sx.pathfind.DStar;
import processing.core.PApplet;
import sx.pathfind.PathFinderTester;

import java.util.PriorityQueue;


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
        Vec2 source = new Vec2(0, 0);
        Vec2 goal = new Vec2(19, 9);


        // Test D* Lite algorithm
        // Init the path finding algorithm
        //DStar dstar = new DStar(source, goal, slam);
        // Init the game
        //PathFinderTester tester = new PathFinderTester(dstar, grid, 50);
        //PApplet.runSketch(new String[]{"ProcessingTest"}, tester);


    }
}