package org.example;

import org.example.dstar.DStar;
import processing.core.PApplet;

import java.util.List;


public class Plotter  extends PApplet
{

    int nCellsX, nCellsY;       // The number of cells in row and column
    int cellSize;               // The size of a cell
    DStar dstar;                // The path finding algorithm
    int fps = 100;              // Frames per second
    int[][] grid;               // The binary grid of obstacles
    List<Vec2> path;            // The current path to the goal found by the algorithm


    public Plotter(DStar dstar, int[][] grid, int cellSize)
    {
        this.dstar = dstar;
        this.cellSize = cellSize;
        this.nCellsX = grid.length;
        this.nCellsY = grid[0].length;
        this.grid = grid;
    }

    public Plotter(DStar dstar, int[][] grid, int cellSize, int fps)
    {
        this.dstar = dstar;
        this.cellSize = cellSize;
        this.nCellsX = grid.length;
        this.nCellsY = grid[0].length;
        this.fps = fps;
        this.grid = grid;
    }


    public void settings()
    {
        size(cellSize * nCellsX, cellSize * nCellsY);
    }

    /* Setup of Processing window */
    public void setup()
    {
        background(220);
        strokeWeight(1);  // Default 4
        dstar.computePath();
        path = dstar.extractPath();
        renderBinaryGrid();
        renderEstimation();
        renderPath();
        renderRobot();
    }

    /* Empty method for a new generation at each frame */
    public void draw()
    {

    }


    // When right button is clicked, the robot make a movement
    // When left button is clicked an obstacle is added / removed
    public void mousePressed ()
    {
        if (mouseButton == RIGHT)
        {
            dstar.step();
            path = dstar.extractPath();
            System.out.println("[INFO] Step to (" + dstar.current.x + "," + dstar.current.y + ")");
        }
        else if (mouseButton == LEFT)
        {
            if (mouseX >= 0 && mouseX < cellSize * nCellsX &&
                    mouseY >= 0 && mouseY < cellSize * nCellsY)
            {
                int x = mouseX / cellSize;
                int y = mouseY / cellSize;
                if (grid[x][y] == 1)
                {
                    grid[x][y] = 0;
                    dstar.setUpdatedSlam(x, y, 0);
                    System.out.println("[INFO] Removed obstacle at (" + x + "," + y + ")");
                } else
                {
                    grid[x][y] = 1;
                    dstar.setUpdatedSlam(x, y, 1);
                    System.out.println("[INFO] Added obstacle at (" + x + "," + y + ")");
                }
                dstar.updateCosts();
                dstar.computePath();
                path = dstar.extractPath();
            }
        }

        background(220);
        strokeWeight(1);  // Default 4
        renderBinaryGrid();
        renderEstimation();
        renderPath();
        renderRobot();
    }


    // Render the binary obstacles map
    public void renderBinaryGrid()
    {
        for (int i = 0; i < grid.length ; i++)
        {
            for (int j = 0; j < grid[0].length; j++)
            {
                if (grid[i][j] == 1)
                {
                    fill(120);
                    stroke(120);
                    rect(i * cellSize, j * cellSize, cellSize, cellSize);
                }
            }
        }
    }


    public void renderEstimation ()
    {
        for (int i = 0; i < grid.length ; i++)
        {
            for (int j = 0; j < grid[0].length; j++)
            {
                textSize(cellSize*0.3f);
                fill(0);
                if (dstar.g[i][j] == Float.POSITIVE_INFINITY)
                    text( "∞", i * cellSize + (float) cellSize / 2, j * cellSize + (float) cellSize / 2);
                else
                    text( String.format("%.1f", dstar.g[i][j]), i * cellSize + (float) cellSize / 2, j * cellSize + (float) cellSize / 2);

                fill(250,0,0);
                if (dstar.rhs[i][j] == Float.POSITIVE_INFINITY)
                    text( "∞", i * cellSize + (float) cellSize / 2, j * cellSize + (float) cellSize / 2);
                else
                    text( String.format("%.1f", dstar.rhs[i][j]), i * cellSize + (float) cellSize / 2, j * cellSize + (float) cellSize / 2);
            }
        }
    }

    // Render the current minimum path
    public void renderPath ()
    {
        for (int i = 1; i < path.size(); i++)
        {
            stroke(0, 150, 0);
            strokeWeight(4);  // Default 4
            float startx = path.get(i - 1).x * cellSize + (float) cellSize / 2;
            float starty = path.get(i - 1).y * cellSize + (float) cellSize / 2;
            float endx = path.get(i).x * cellSize + (float) cellSize / 2;
            float endy = path.get(i).y * cellSize + (float) cellSize / 2;
            line(startx, starty, endx, endy);
        }
    }

    // Render the robot on the grid map
    public void renderRobot ()
    {
        stroke(100, 0, 0);
        fill(100, 0, 0);
        ellipse(dstar.current.x * cellSize + (float) cellSize / 2, dstar.current.y* cellSize + (float) cellSize / 2, (float) cellSize / 3, (float) cellSize/3);
    }

}

