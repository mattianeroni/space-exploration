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


    public void setup()
    {
        background(220);
        strokeWeight(1);  // Default 4
        dstar.step();
        dstar.computePath();
        List<Vec2> path = dstar.extractPath();
        renderBinaryGrid();
        renderPath(path);
    }

    public void draw()
    {
        background(220);
        strokeWeight(1);  // Default 4
        dstar.step();
        dstar.computePath();
        List<Vec2> path = dstar.extractPath();
        renderBinaryGrid();
        renderPath(path);

    }


    // When right button is clicked, the robot make a movement
    // When left button is clicked an obstacle is added / removed
    public void mousePressed ()
    {
        if (mouseButton == RIGHT)
        {
            dstar.step();
            System.out.println("[INFO] Step to (" + dstar.current.x + "," + dstar.current.y + ")");
        }
        else if (mouseButton == LEFT)
        {
            if (mouseX >= 0 && mouseX < cellSize * nCellsX &&
                    mouseY >= 0 && mouseY < cellSize * nCellsY) {
                int x = mouseX / cellSize;
                int y = mouseY / cellSize;
                if (grid[x][y] == 1) {
                    grid[x][y] = 0;
                    dstar.removeObstacle(new Vec2(x,y));
                    System.out.println("[INFO] Removed obstacle at (" + x + "," + y + ")");
                } else {
                    grid[x][y] = 1;
                    dstar.addObstacle(new Vec2(x,y));
                    System.out.println("[INFO] Added obstacle at (" + x + "," + y + ")");
                }
            }
        }
    }


    // Render the binary obstacles map
    public void renderBinaryGrid()
    {
        for (int i = 0; i < grid.length ; i++){
            for (int j = 0; j < grid[0].length; j++){
                if (grid[i][j] == 1){
                    fill(120);
                    rect(i * cellSize, j * cellSize, cellSize, cellSize);
                }
            }
        }
    }

    // Render the current minimum path
    public void renderPath (List<Vec2> path)
    {
        for (int i = 1; i < path.size(); i++){
            fill(0,200,0);
            line(path.get(i - 1).x, path.get(i - 1).y, path.get(i).x, path.get(i).y);
        }

    }


}

