package sx.pathfind;

import sx.Vec2;

import java.util.Objects;

public class AStarEstimation
{

    /*

        An instance of this class represents an estimation associated with
        a position.
        It keeps information about the position, the distance to cover to reach that position,
        and the estimation of the distance to the destination (i.e., goal).

        NOTE: As soon as the current robot position or the goal changes this estimation
        expires and is not reliable anymore.

    */

    public float cost;                          // Minimum distance from the current robot position to the position
    public float h;                             // Estimation of the distance to cover to reach the goal
    public Vec2 position;                       // Position for which values have been estimated


    public AStarEstimation (Vec2 position, float cost, float h)
    {
        this.position = position;
        this.cost = cost;
        this.h = h;
    }

}
