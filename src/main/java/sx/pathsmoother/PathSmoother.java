package sx.pathsmoother;

import sx.Coordinate;
import java.util.LinkedList;



public interface PathSmoother
{

    // Method to smooth a path
    LinkedList<Coordinate> smooth (LinkedList<Coordinate> path);

}
