package sx.frontier;

import sx.Vec2i;

import java.util.List;

public interface FrontierDetector
{

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
