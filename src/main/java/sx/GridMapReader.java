package sx;

import java.io.File;
import java.io.IOException;
import java.util.Scanner;

public class GridMapReader
{

    /* Read a new grid map */
    public static int[][] readMap (File file, int sizeX, int sizeY) throws IOException
    {

        int[][] grid = new int[sizeX][sizeY];

        Scanner scanner = new Scanner(file);
        String[] pixels = scanner.nextLine().split(",");
        int counter = 0;

        for (int i = 0; i < sizeX; i++)
        {
            for (int j = 0; j < sizeY; j++)
            {

                grid[i][j] = Integer.parseInt(pixels[counter]);
                counter++;

            }
        }
        return grid;
    }


    /*
        Convert ROS grid map to obstacles binary grid map
        :param grid: the ROS grid map
        :param bravery: over this value elements of ROS grid map are considered obstacles
    */
    public static int[][] makeBinaryGridMap (int[][] grid, int bravery)
    {

        int[][] Bgrid = new int[grid.length][grid[0].length];

        for (int i = 0; i < grid.length; i++)
            for (int j = 0; j < grid[0].length; j++)
                if (grid[i][j] > bravery)
                    Bgrid[i][j] = 1;

        return Bgrid;
    }

}
