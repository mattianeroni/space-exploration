package sx.frontier;

import sx.Vec2i;

import java.util.List;

public interface FrontierDetector
{

    /*
    ================================================================================================================

        Interface for frontier detection algorithms. All algorithms should implement this
        interface in order to be tested by the FrontierDetectorTester.

        The currently implemented ones are:
            - An instance of this class represents a customized implementation
              of Wavefront Frontier Detection algorithm.
              Which in enhanced using a QuadTree to a faster detection of the frontiers
              that should be extended.

    ================================================================================================================
    */

    /* Method to require a new computation of frontiers */
    void computeDetection();


    /* Returns the robot position on the grid map */
    Vec2i getRobotPosition();


    /* Method to reset the computations and the information stored until that moment */
    void reset();


    /* Method to update the environment as known by the frontier detector */
    void updateSlam(int x, int y, int value);


    /* Method to get the frontiers */
    List<Frontier> getFrontiers();

}
