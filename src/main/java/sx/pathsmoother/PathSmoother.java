package sx.pathsmoother;

import sx.Vec2f;
import java.util.LinkedList;



public interface PathSmoother
{

    // Method to smooth a path
    LinkedList<Vec2f> smooth (LinkedList<Vec2f> path);

}
