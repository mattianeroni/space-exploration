package sx;

import sx.dstar.DStar;
import sx.dstar.exceptions.NoPathFound;
import processing.core.PApplet;

import java.text.DateFormat;
import java.text.SimpleDateFormat;
import java.util.Date;


public class Plotter  extends PApplet
{

    int nCellsX, nCellsY;       // The number of cells in row and column
    int cellSize;               // The size of a cell
    DStar dstar;                // The path finding algorithm
    int fps = 100;              // Frames per second
    int[][] grid;               // The binary grid of obstacles
    DateFormat time;            // The timeprint format


    public Plotter(DStar dstar, int[][] grid, int cellSize)
    {
        this.dstar = dstar;
        this.cellSize = cellSize;
        this.nCellsX = grid.length;
        this.nCellsY = grid[0].length;
        this.grid = grid;
        this.time = new SimpleDateFormat("HH:mm:ss.SSS");
    }

    public Plotter(DStar dstar, int[][] grid, int cellSize, int fps)
    {
        this.dstar = dstar;
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
        dstar.computePath(false);
        try{
            dstar.extractPath();
        } catch (NoPathFound e) {
            throw new RuntimeException(e);
        }
        renderBinaryGrid();
        renderEstimation();
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
            dstar.step();
            System.out.println("[" + time.format(new Date()) + "][INFO] Step to (" + dstar.current.x + "," + dstar.current.y + ")");
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
                    dstar.updateSlam(new Vec2(x, y), 0);
                    System.out.println("[" + time.format(new Date()) + "][INFO] Removed obstacle at (" + x + "," + y + ")");
                } else
                {
                    grid[x][y] = 1;
                    dstar.updateSlam(new Vec2(x, y), 1);
                    System.out.println("[" + time.format(new Date()) + "][INFO] Added obstacle at (" + x + "," + y + ")");
                }

                dstar.computePath(false);
                try {
                    dstar.extractPath();
                    System.out.println("[" + time.format(new Date()) + "][INFO] New path found");
                } catch (NoPathFound e) {
                    dstar.reset(); // = new DStar(dstar.current, dstar.goal, dstar.slam);
                    dstar.computePath(false);
                    try {
                        dstar.extractPath();
                        System.out.println("[" + time.format(new Date()) + "][INFO] New path found after re-computation");
                    } catch (NoPathFound ex){
                        throw new RuntimeException(ex);
                    }
                }
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
                textSize(cellSize * 0.2f);
                fill(0);
                /*if (dstar.g[i][j] == Float.POSITIVE_INFINITY)
                    text( "∞", i * cellSize + (float) cellSize / 2, j * cellSize + (float) cellSize / 2);
                else
                    text( String.format("%.1f", dstar.g[i][j]), i * cellSize + (float) cellSize / 2, j * cellSize + (float) cellSize / 2);

                fill(250,0,0);*/
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
        Vec2 cNode = dstar.current;
        for (int i = 0; i < dstar.path.size(); i++)
        {
            stroke(0, 150, 0);
            strokeWeight(4);  // Default 4
            float startx = cNode.x * cellSize + (float) cellSize / 2;
            float starty = cNode.y * cellSize + (float) cellSize / 2;
            float endx = dstar.path.get(i).x * cellSize + (float) cellSize / 2;
            float endy = dstar.path.get(i).y * cellSize + (float) cellSize / 2;
            line(startx, starty, endx, endy);
            cNode = dstar.path.get(i);
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

