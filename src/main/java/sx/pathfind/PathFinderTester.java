package sx.pathfind;

import sx.Vec2f;
import sx.Vec2i;
import processing.core.PApplet;
import sx.GridToCoordinates;
import sx.pathsmoother.PathSmoother;

import java.text.DateFormat;
import java.text.SimpleDateFormat;
import java.util.Date;
import java.util.LinkedList;



public class PathFinderTester  extends PApplet
{

    int nCellsX, nCellsY;           // The number of cells in row and column
    int cellSize;                   // The size of a cell
    PathFinder pathFinder;          // The path finding algorithm
    int[][] grid;                   // The binary grid of obstacles
    DateFormat time;                // The timeprint format
    PathSmoother pathSmoother;      // The path smoothing algorithm
    //int fps = 100;                // Frames per second


    public PathFinderTester(PathFinder pathFinder, int[][] grid, int cellSize)
    {
        this.pathFinder = pathFinder;
        this.cellSize = cellSize;
        this.nCellsX = grid.length;
        this.nCellsY = grid[0].length;
        this.grid = grid;
        this.pathSmoother = null;
        this.time = new SimpleDateFormat("HH:mm:ss.SSS");
    }



    public PathFinderTester(PathFinder pathFinder, PathSmoother pathSmoother, int[][] grid, int cellSize)
    {
        this.pathFinder = pathFinder;
        this.cellSize = cellSize;
        this.nCellsX = grid.length;
        this.nCellsY = grid[0].length;
        this.grid = grid;
        this.pathSmoother = pathSmoother;
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
        renderSmoothedPath();
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
                    pathFinder.updateSlam(new Vec2i(x, y), 0);
                    System.out.println("[" + time.format(new Date()) + "][INFO] Removed obstacle at (" + x + "," + y + ")");

                } else
                {

                    // Add obstacle
                    grid[x][y] = 1;
                    pathFinder.updateSlam(new Vec2i(x, y), 1);
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
        renderSmoothedPath();
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
                    //rect(i * cellSize, j * cellSize, cellSize, cellSize);
                    ellipse(i * cellSize + (float) cellSize / 2.0f, j * cellSize + (float) cellSize / 2.0f, (float) cellSize * 0.8f, (float) cellSize * 0.8f);
                }
            }
        }
    }


    /* Render the current minimum path */
    public void renderPath ()
    {
        Vec2i cNode = pathFinder.getCurrent();
        LinkedList<Vec2i> path = pathFinder.getPath();
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


    /* Render the smoothed path */
    public void renderSmoothedPath ()
    {
        if (pathSmoother == null)
            return;

        GridToCoordinates gtc = new GridToCoordinates((float) cellSize);
        LinkedList<Vec2f> path = pathSmoother.smooth(gtc.convert(pathFinder.getPath()));
        Vec2f cNode = path.getFirst();
        float cellSizef = (float) cellSize;

        for (int i = 1; i < path.size(); i++)
        {
            stroke(0, 0, 150);
            strokeWeight(4);  // Default 4
            line(cNode.x, cNode.y, path.get(i).x, path.get(i).y);
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
    }*/


}

