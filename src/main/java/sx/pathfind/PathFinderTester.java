package sx.pathfind;

import sx.Vec2;
import sx.pathfind.PathFinder;
import processing.core.PApplet;

import java.text.DateFormat;
import java.text.SimpleDateFormat;
import java.util.Date;
import java.util.LinkedList;


public class PathFinderTester  extends PApplet
{

    int nCellsX, nCellsY;       // The number of cells in row and column
    int cellSize;               // The size of a cell
    PathFinder pathFinder;      // The path finding algorithm
    int fps = 100;              // Frames per second
    int[][] grid;               // The binary grid of obstacles
    DateFormat time;            // The timeprint format


    public PathFinderTester(PathFinder pathFinder, int[][] grid, int cellSize)
    {
        this.pathFinder = pathFinder;
        this.cellSize = cellSize;
        this.nCellsX = grid.length;
        this.nCellsY = grid[0].length;
        this.grid = grid;
        this.time = new SimpleDateFormat("HH:mm:ss.SSS");
    }

    public PathFinderTester(PathFinder pathFinder, int[][] grid, int cellSize, int fps)
    {
        this.pathFinder = pathFinder;
        this.cellSize = cellSize;
        this.nCellsX = grid.length;
        this.nCellsY = grid[0].length;
        this.fps = fps;
        this.grid = grid;
        this.time = new SimpleDateFormat("HH:mm:ss.SSS");
    }


    public void settings()
    {
        size(cellSize * nCellsX, cellSize * nCellsY);
    }


    /* Setup of Processing window */
    public void setup()
    {
        background(220);
        strokeWeight(1);        // Default 4
        pathFinder.computeStartingPath();
        renderBinaryGrid();
        renderPath();
        renderRobot();
    }

    /* Empty method for a new generation at each frame */
    public void draw()
    {

    }


    /*
     When right button is clicked, the robot make a movement.
     When left button is clicked an obstacle is added / removed.
    */
    public void mousePressed() {
        if (mouseButton == RIGHT)
        {

            pathFinder.step();
            System.out.println("[" + time.format(new Date()) + "][INFO] Step to (" + pathFinder.getCurrent().x + "," + pathFinder.getCurrent().y + ")");

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

                    // Remove obstacle
                    grid[x][y] = 0;
                    pathFinder.updateSlam(new Vec2(x, y), 0);
                    System.out.println("[" + time.format(new Date()) + "][INFO] Removed obstacle at (" + x + "," + y + ")");

                } else
                {

                    // Add obstacle
                    grid[x][y] = 1;
                    pathFinder.updateSlam(new Vec2(x, y), 1);
                    System.out.println("[" + time.format(new Date()) + "][INFO] Added obstacle at (" + x + "," + y + ")");

                }

                // Compute new minimum path
                pathFinder.computePath();
                System.out.println("[" + time.format(new Date()) + "][INFO] New path found");

            }
        }

        background(220);
        strokeWeight(1);  // Default 4
        renderBinaryGrid();
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


    /* Render the current minimum path */
    public void renderPath ()
    {
        Vec2 cNode = pathFinder.getCurrent();
        LinkedList<Vec2> path = pathFinder.getPath();
        for (int i = 1; i < pathFinder.getPath().size(); i++)
        {
            stroke(0, 150, 0);
            strokeWeight(4);  // Default 4
            float startx = cNode.x * cellSize + (float) cellSize / 2;
            float starty = cNode.y * cellSize + (float) cellSize / 2;
            float endx = path.get(i).x * cellSize + (float) cellSize / 2;
            float endy = path.get(i).y * cellSize + (float) cellSize / 2;
            line(startx, starty, endx, endy);
            cNode = path.get(i);
        }
    }



    /* Render the robot on the grid map */
    public void renderRobot ()
    {
        stroke(100, 0, 0);
        fill(100, 0, 0);
        ellipse(pathFinder.getCurrent().x * cellSize + (float) cellSize / 2, pathFinder.getCurrent().y* cellSize + (float) cellSize / 2, (float) cellSize / 3, (float) cellSize/3);
    }



    /*
      Method to render the RHS and G values associated with the grid map.
      This method works only for D* inspired algorithms.

    public void renderEstimation ()
    {
        for (int i = 0; i < grid.length ; i++)
        {
            for (int j = 0; j < grid[0].length; j++)
            {
                textSize(cellSize * 0.2f);
                fill(0);

                if (pathFinder.rhs[i][j] == Float.POSITIVE_INFINITY)
                    text( "∞", i * cellSize + (float) cellSize / 2, j * cellSize + (float) cellSize / 2);
                else
                    text( String.format("%.1f", pathFinder.rhs[i][j]), i * cellSize + (float) cellSize / 2, j * cellSize + (float) cellSize / 2);

            }
        }
    }
    */

}

