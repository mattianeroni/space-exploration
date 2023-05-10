package sx.pathsmoother;

import sx.Vec2f;
import java.util.LinkedList;



public interface PathSmoother
{
    /*
    ================================================================================================================

        Interface of path smoothing algorithms.

        The currently implemented ones are:
            - A GeometricSmoother smoothing the path through a geometric approach.
            - A GradientAscent iterative approach (NOT WORKING WELL)

    ================================================================================================================
    */

    // Method to smooth a path
    LinkedList<Vec2f> smooth (LinkedList<Vec2f> path);

}
