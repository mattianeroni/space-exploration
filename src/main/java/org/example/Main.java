package org.example;

import org.example.dstar.DStar;
import org.example.dstar.HeapNode;
import processing.core.PApplet;
import java.io.File;
import java.io.IOException;
import java.util.*;


public class Main
{


    /* Read a new grid map */
    public static int[][] readMap (File file, int sizeX, int sizeY) throws IOException
    {

        int[][] grid = new int[sizeX][sizeY];

        Scanner scanner = new Scanner(file);
        String[] pixels = scanner.nextLine().split(",");
        int counter = 0;

        for (int i = 0; i < sizeX; i++)
        {
            for (int j = 0; j < sizeY; j++)
            {

                grid[i][j] = Integer.parseInt(pixels[counter]);
                counter++;

            }
        }
        return grid;
    }


    /*
        Convert ROS grid map to obstacles binary grid map
        :param grid: the ROS grid map
        :param bravery: over this value elements of ROS grid map are considered obstacles
    */
    public static int[][] makeBinaryGridMap (int[][] grid, int bravery)
    {

        int[][] Bgrid = new int[grid.length][grid[0].length];

        for (int i = 0; i < grid.length; i++)
        {
            for (int j = 0; j < grid[0].length; j++)
            {

                if (grid[i][j] > bravery)
                    Bgrid[i][j] = 1;

            }
        }
        return Bgrid;
    }


    // The size of the grid map
    static int ROWS = 100;
    static int COLS = 100;

    // The bravery. Over this value of accuracy, elements in ROS map
    // are considered obstacles.
    static int BRAVERY = 90;


    public static void main(String[] args) throws IOException
    {

        // Create the grid map
        int[][] grid = new int[ROWS][COLS];   //readMap(new File("map.csv"), ROWS, COLS);

        // Translate the grid map into a binary obstacles map
        // grid = makeBinaryGridMap(grid, BRAVERY);

        // Source and target points
        Vec2 source = new Vec2(0, 0);
        Vec2 goal = new Vec2(90, 90);

        // https://github.com/zhm-real/PathPlanning/blob/master/Search_based_Planning/Search_2D/D_star_Lite.py

        Map<Vec2, Integer> map = new HashMap<>();
        map.put(source, 0);
        //map.put(goal, 1);
        System.out.println(map.get(goal));

        DStar dstar = new DStar(source, goal, grid);
        //Plotter plt = new Plotter(dstar, grid, 3);
        //PApplet.runSketch(new String[]{"ProcessingTest"}, plt);

    }
}