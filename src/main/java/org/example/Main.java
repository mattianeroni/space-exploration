package org.example;

import org.example.dstar.DStar;
import org.example.dstar.HeapNode;
import processing.core.PApplet;
import java.io.File;
import java.io.IOException;
import java.util.*;


public class Main
{
    // The size of the grid map
    static int ROWS = 10;
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
        Vec2 goal = new Vec2(9, 9);

        // Init the path finding algorithm
        DStar dstar = new DStar(source, goal, slam);


        // Init the game
        Plotter plt = new Plotter(dstar, grid, 50);
        PApplet.runSketch(new String[]{"ProcessingTest"}, plt);

    }
}