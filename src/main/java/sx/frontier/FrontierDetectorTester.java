package sx.frontier;

import processing.core.PApplet;
import sx.Vec2i;

import java.text.DateFormat;
import java.text.SimpleDateFormat;
import java.util.Date;
import java.util.List;
import java.util.concurrent.ThreadLocalRandom;


public class FrontierDetectorTester extends PApplet
{


    public FrontierDetector frontierDetector;    // The frontier detection algorithm to validate
    public int[][] grid;                         // The grid under exploration --i.e., 0 free explored space,
                                                 // 1 obstacle, -1 unknown

    int nCellsX, nCellsY;                        // The number of cells in row and column
    int cellSize;                                // The size of a cell

    DateFormat time;                             // The time print format for logs


    public FrontierDetectorTester(FrontierDetector frontierDetector, int[][] grid, int cellSize)
    {
        this.frontierDetector = frontierDetector;
        this.cellSize = cellSize;
        this.nCellsX = grid.length;
        this.nCellsY = grid[0].length;
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
        System.out.println("[" + time.format(new Date()) + "][INFO] Frontier calculation started");
        frontierDetector.computeDetection();
        System.out.println("[" + time.format(new Date()) + "][INFO] Frontier calculation concluded");
        renderGrid();
        renderFrontiers();
    }

    /* Empty method for a new generation at each frame */
    public void draw()
    {

    }




    /*
        Draw obstacle or expand the explored area by dragging the mouse.
        If left button is used, the explored area is expanded.
        If right button is used, obstacles are added to the grid.
     */
    public void mouseDragged ()
    {
        // Check if we are inside the grid
        if (mouseX >= 0 && mouseX < cellSize * nCellsX && mouseY >= 0 && mouseY < cellSize * nCellsY)
        {
            // Get current cell fro mouse position
            int x = mouseX / cellSize;
            int y = mouseY / cellSize;

            if (mouseButton == LEFT && grid[x][y] == -1) {

                grid[x][y] = 0;
                frontierDetector.updateSlam(x, y, 0);
                System.out.println("[" + time.format(new Date()) + "][INFO] Visited (" + x + "," + y + ")");

            } else if (mouseButton == RIGHT && grid[x][y] == -1) {

                grid[x][y] = 1;
                frontierDetector.updateSlam(x, y, 1);
                System.out.println("[" + time.format(new Date()) + "][INFO] Obstacle added at (" + x + "," + y + ")");
            }

            renderGrid();

        }
    }


    /*
        Draw obstacle or expand the explored area by pressing the mouse.
        If left button is used, the explored area is expanded.
        If right button is used, obstacles are added to the grid.
     */
    public void mousePressed ()
    {
        // Check if we are inside the grid
        if (mouseX >= 0 && mouseX < cellSize * nCellsX && mouseY >= 0 && mouseY < cellSize * nCellsY)
        {
            // Get current cell fro mouse position
            int x = mouseX / cellSize;
            int y = mouseY / cellSize;

            if (mouseButton == LEFT && grid[x][y] == -1) {

                grid[x][y] = 0;
                frontierDetector.updateSlam(x, y, 0);
                System.out.println("[" + time.format(new Date()) + "][INFO] Visited (" + x + "," + y + ")");

            } else if (mouseButton == RIGHT && grid[x][y] == -1) {

                grid[x][y] = 1;
                frontierDetector.updateSlam(x, y, 1);
                System.out.println("[" + time.format(new Date()) + "][INFO] Obstacle added at (" + x + "," + y + ")");
            }

            renderGrid();

        }
    }

    public void keyPressed()
    {
        if (keyPressed && keyCode == ' ')
        {
            System.out.println("[" + time.format(new Date()) + "][INFO] Frontier calculation started");
            frontierDetector.computeDetection();
            System.out.println("[" + time.format(new Date()) + "][INFO] Frontier calculation concluded");
            renderGrid();
            renderFrontiers();
        }
    }



    /* Render the grid under exploration */
    public void renderGrid()
    {
        background(220);
        float ballSize = (float) cellSize * 0.8f;

        for (int i = 0; i < grid.length ; i++)
        {
            for (int j = 0; j < grid[0].length; j++)
            {
                if (grid[i][j] == 1)
                {
                    fill(90);
                    stroke(90);
                    strokeWeight(1);
                    ellipse(i * cellSize + (float) cellSize / 2.0f, j * cellSize + (float) cellSize / 2.0f, ballSize, ballSize);
                } else if (grid[i][j] == -1)
                {
                    fill(120);
                    stroke(120);
                    strokeWeight(1);
                    ellipse(i * cellSize + (float) cellSize / 2.0f, j * cellSize + (float) cellSize / 2.0f, ballSize, ballSize);
                }
            }
        }
    }



    /* Render the bounding boxes around the frontiers and color frontier cells */
    public void renderFrontiers()
    {
        List<Frontier> frontierList = frontierDetector.getFrontiers();
        float ballSize = (float) cellSize * 0.8f;

        System.out.println(frontierList.size());

        for (Frontier frontier : frontierList)
        {
            int r = ThreadLocalRandom.current().nextInt(0,  255 + 1);
            int b = ThreadLocalRandom.current().nextInt(0,  255 + 1);
            int g = ThreadLocalRandom.current().nextInt(0,  255 + 1);

            for (Vec2i i : frontier.cells)
            {
                fill(r, b, g);
                stroke(r, b, g);
                strokeWeight(1);
                ellipse(i.x * cellSize + (float) cellSize / 2.0f, i.y * cellSize + (float) cellSize / 2.0f, ballSize, ballSize);
            }

            noFill();
            strokeWeight(3);        // Default 4
            rect((float) frontier.box.getMinX()* cellSize + (float) cellSize / 2.0f, (float) frontier.box.getMinY()* cellSize + (float) cellSize / 2.0f,
                    (float) ((frontier.box.getMaxX() - frontier.box.getMinX()) * cellSize), (float) ( (frontier.box.getMaxY() - frontier.box.getMinY())*cellSize ));
            strokeWeight(1);        // Default 4
        }
    }

}
