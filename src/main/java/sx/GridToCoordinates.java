package sx;

import java.util.LinkedList;



public class GridToCoordinates
{

    /*
        Code to convert a path on a grid map into a path made of coordinates in a
        2-dimensional space.
    */

    LinkedList<Vec2f> path;     // Final computed path of spatial coordinates
    float cellSize;             // The size of a cell

    public GridToCoordinates (float cellSize)
    {
        this.cellSize = cellSize;
        this.path = new LinkedList<>();
    }


    /* Translate a path expressed in cells on the grid map in a path of spatial coordinates */
    public LinkedList<Vec2f> convert (LinkedList<Vec2i> originalPath)
    {
        for (Vec2i position : originalPath)
            path.add( new Vec2f(position.x * cellSize + (cellSize / 2.0f), position.y * cellSize + (cellSize / 2.0f)) );
        return path;
    }

}
