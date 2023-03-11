package org.example;

import processing.core.PApplet;


public class Plotter  extends PApplet {
    // Define the grid size
    int nCellsX, nCellsY;
    int cellSize;

    // Frames per second
    int fps = 100;

    // The grid
    int[][] grid;

    public Plotter(int[][] grid, int cellSize){
        this.cellSize = cellSize;
        this.nCellsX = grid.length;
        this.nCellsY = grid[0].length;
        this.grid = grid;
    }

    public Plotter(int[][] grid, int cellSize, int fps){
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
        fill(200, 0 ,0);
        strokeWeight(1);  // Default 4

        // Render grid
        renderBinaryGrid();
    }

    public void draw(){
        renderBinaryGrid();
    }

    // Add or remove obstacles by dragging the mouse
    public void mouseDragged() {
        if (mouseX >= 0 && mouseX < cellSize * nCellsX &&
        mouseY >= 0 && mouseY < cellSize * nCellsY) {
            int x = mouseX / cellSize;
            int y = mouseY / cellSize;
            if (grid[x][y] == 1) {
                grid[x][y] = 0;
            } else {
                grid[x][y] = 1;
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

