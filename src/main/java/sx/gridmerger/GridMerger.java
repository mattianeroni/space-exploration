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



    static int[][] translateGrid (int[][] grid, Vec2i translation)
    {
        // Init a new grid
        int[][] newGrid = new int[grid.length][grid[0].length];

        // Not covered positions are considered unknown
        for (int[] ints : newGrid) Arrays.fill(ints, -1);

        // Compute translation
        for (int x = 0; x < grid.length; x++) {
            for (int y = 0; y < grid[0].length; y++) {

                int tx = x + translation.x;
                int ty = y + translation.y;

                if (tx >= 0 && tx < grid.length && ty >= 0 && ty < grid[0].length)
                    newGrid[tx][ty] = grid[x][y];

            }
        }

        return newGrid;
    }

}