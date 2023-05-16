package sx.gridmerger;

import sx.Vec2i;
import java.util.Arrays;


public interface GridMerger
{

    /*
    ================================================================================================================

        Interface of grids merging algorithms. All grids merging algorithms should implement this
        interface in order to be tested and executed.

        NOTE: This interface also provide standard methods for rotation and translation.

    ================================================================================================================
    */


    /* Rotate a position according to a rotation center and an angle expressed in radians */
    static Vec2i rotate (Vec2i position, Vec2i center, float angle)
    {
        float s = (float) Math.sin(angle);
        float c = (float) Math.cos(angle);

        float newX = (position.x - center.x) * c - (position.y - center.y) * s + center.x;
        float newY = (position.x - center.x) * s + (position.y - center.y) * c + center.y;

        return new Vec2i( Math.round(newX), Math.round(newY));
    }



    /*
        Rotate a full grid map according to a rotation center and an angle expressed in radians
        NOTE: The new grid map is always of the same shape of the first one. We never resize the
        matrix, and the values outside borders are lost.
    */
    static int[][] rotateGrid (int[][] grid, Vec2i center, float angle)
    {
        // Init a new grid
        int[][] newGrid = new int[grid.length][grid[0].length];

        // Not covered positions are considered unknown
        for (int[] ints : newGrid) Arrays.fill(ints, -1);
        newGrid[center.x][center.y] = grid[center.x][center.y];

        float s = (float) Math.sin(angle);
        float c = (float) Math.cos(angle);

        for (int x = 0; x < grid.length; x++)
        {
            for (int y = 0; y < grid[0].length; y++)
            {
                // Unknown regions does not need to be considered
                if (grid[x][y] == -1)
                    continue;

                // Center does not need to be rotated
                if (x == center.x && y == center.y)
                    continue;

                int newx = Math.round((x - center.x) * c - (y - center.y) * s + center.x);
                int newy = Math.round((x - center.x) * s + (y - center.y) * c + center.y);

                // Only values inside boarders are accepted
                if (newx >= 0 && newx < grid.length && newy >= 0 && newy < grid[0].length)
                    newGrid[newx][newy] = grid[x][y];

            }
        }

        return newGrid;
    }



    /*
        Translate a full grid.
        NOTE: The values outside borders are lost.
    */
    static int[][] translateGrid (int[][] grid, Vec2i translation)
    {
        // Init a new grid
        int[][] newGrid = new int[grid.length][grid[0].length];

        // Not covered positions are considered unknown
        for (int[] ints : newGrid) Arrays.fill(ints, -1);

        // Define boundaries before to make less iterations
        int minX = Math.max(translation.x, 0);
        int minY = Math.max(translation.y, 0);
        int maxX = Math.min(grid.length, grid.length + translation.x);
        int maxY = Math.min(grid[0].length, grid[0].length + translation.y);

        // Compute translation
        for (int x = minX; x < maxX; x++)
            for (int y = minY; y < maxY; y++)
                newGrid[x][y] = grid[x - translation.x][y - translation.y];

        return newGrid;
    }



    /*
        Translate and rotate at the same time a grid map.

        NOTE: The values outside borders are lost.
        NOTE: This is more efficient than rotating and translating in two different steps.
    */
    static int[][] transformGrid (int[][] grid, Transform transform)
    {
        // Init a new grid
        int[][] newGrid = new int[grid.length][grid[0].length];

        // Not covered positions are considered unknown
        for (int[] ints : newGrid) Arrays.fill(ints, -1);

        float s = (float) Math.sin(transform.angle);
        float c = (float) Math.cos(transform.angle);

        // Compute translation
        for (int x = 0; x < grid.length; x++)
        {
            for (int y = 0; y < grid[0].length; y++)
            {
                // Unknown regions does not need to be considered
                if (grid[x][y] == -1)
                    continue;

                // Rotate
                int rotatedX = Math.round((x - transform.center.x) * c - (y - transform.center.y) * s + transform.center.x);
                int rotatedY = Math.round((x - transform.center.x) * s + (y - transform.center.y) * c + transform.center.y);

                // Translate
                int finalX = rotatedX + transform.translation.x;
                int finalY = rotatedY + transform.translation.y;

                if (finalX >= 0 && finalX < grid.length && finalY >= 0 && finalY < grid[0].length)
                    newGrid[finalX][finalY] = grid[x][y];

            }
        }

        return newGrid;
    }


}
