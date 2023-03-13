package org.example;

import org.example.dstar.DStar;
import processing.core.PApplet;


public class Plotter  extends PApplet {
    // Define the grid size
    int nCellsX, nCellsY;
    int cellSize;

    // The path finding algorithm
    DStar dstar = null;

    // Frames per second
    int fps = 100;

    // The grid
    int[][] grid;

    public Plotter(DStar dstar, int[][] grid, int cellSize){
        this.dstar = dstar;
        this.cellSize = cellSize;
        this.nCellsX = grid.length;
        this.nCellsY = grid[0].length;
        this.grid = grid;
    }

    public Plotter(DStar dstar, int[][] grid, int cellSize, int fps){
        this.dstar = dstar;
        this.cellSize = cellSize;
        this.nCellsX = grid.length;
        this.nCellsY = grid[0].length;
        this.fps = fps;
        this.grid = grid;
    }

    public void settings(){
        size(cellSize * nCellsX, cellSize * nCellsY);
    }

    public void setup(){
        background(220);
        //fill(120);
        strokeWeight(1);  // Default 4

        // Render grid
        renderBinaryGrid();
    }

    public void draw(){background(220);
        background(220);
        strokeWeight(1);  // Default 4
        renderBinaryGrid();
    }


    // When right button is clicked, the robot make a movement
    // When left button is clicked an obstacle is added / removed
    public void mousePressed () {
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
    public void renderBinaryGrid(){
        for (int i = 0; i < grid.length ; i++){
            for (int j = 0; j < grid[0].length; j++){
                if (grid[i][j] == 1){
                    fill(120);
                    rect(i * cellSize, j * cellSize, cellSize, cellSize);
                }
            }
        }
    }

}

