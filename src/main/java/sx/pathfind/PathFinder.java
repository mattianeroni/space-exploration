package sx.pathfind;

import sx.Vec2i;
import sx.pathfind.exceptions.NoPathFound;

import java.util.LinkedList;



public interface PathFinder
{

    /*
    ================================================================================================================

        Interface of path finding algorithms. All path finding algorithms should implement this
        interface in order to be tested by the PathFinderTester.

        The currently implemented ones are:
            - D* Lite (Koenig, S., & Likhachev, M. (2002). D* lite. Aaai/iaai, 15, 476-483)
            - A* (Hart, P., Nilsson, N., & Raphael, B. (1968). A Formal Basis for the Heuristic Determination of
                Minimum Cost Paths. IEEE Transactions on Systems Science and Cybernetics, 4(2), 100–107)

    ================================================================================================================
    */

    /* Method to extract the current minimum path to the goal */
    void extractPath() throws NoPathFound;


    /* Method to compute the minimum path to the goal */
    void computePath();


    /*
        Method to compute the starting path at the first iteration.
        It can be the same of computePath.
     */
    void computeStartingPath();


    /* Method to update the algorithm SLAM --i.e., the knowledge the algorithm has of the environment */
    void updateSlam(Vec2i position, int value);


    /* Method to update the robot position and move it to the next planned position */
    void step();


    /* Method to return the current position occupied by the robot known by the algorithm */
    Vec2i getCurrent();


    /* Method to get the current minimum path */
    LinkedList<Vec2i> getPath();

}
