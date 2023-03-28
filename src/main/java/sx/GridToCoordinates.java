package sx;

import sx.Coordinate;
import sx.Vec2;
import java.util.LinkedList;



public class GridToCoordinates
{

    /*
        Code to convert a path on a grid map into a path made of coordinates in a
        2-dimensional space.
    */

    LinkedList<Coordinate> path;
    float cellSize;

    public GridToCoordinates (float cellSize)
    {
        this.cellSize = cellSize;
        this.path = new LinkedList<>();
    }


    public LinkedList<Coordinate> convert (LinkedList<Vec2> originalPath)
    {
        for (Vec2 position : originalPath)
            path.add( new Coordinate(position.x * cellSize + (cellSize / 2.0f), position.y * cellSize + (cellSize / 2.0f)) );
        return path;
    }

}
