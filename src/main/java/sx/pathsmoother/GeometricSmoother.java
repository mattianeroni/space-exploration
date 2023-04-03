package sx.pathsmoother;

import sx.Vec2f;
import java.util.*;


public class GeometricSmoother implements PathSmoother
{
    /*
        This class contains an implementation of the Gradient Ascent algorithm to smooth a path.

        The path is smoothed adopting a geometric approach based on angles and lines.

        Please, note this is just a path smoother, not a trajectory planner considering inertia,
        physics components, and collisions detection.
    */

    public float lookAhead;                 // How far the smoother look for understanding the overall direction
                                            // NOTE: The higher is this value, the smoother is the computed path
                                            // and the smaller is the correspondence with the path computed by the
                                            // path finding algorithm.
    public float granularity;               // The distance between consecutive positions in the smoother path.

    public LinkedList<Vec2f> smoothedPath;  // The last smoothed path
    private Vec2f currentPos;               // The current position of the robot on the smoothed path under construction
    private Vec2f finalTarget;              // The final position to reach


    public GeometricSmoother (float lookAhead, float granularity)
    {
        this.lookAhead = lookAhead;
        this.granularity = granularity;
        this.smoothedPath = new LinkedList<>();
    }


    @Override
    public LinkedList<Vec2f> smooth (LinkedList<Vec2f> pathToSmooth)
    {
        // Initialise all class attributes to use them in other method easily
        smoothedPath = new LinkedList<>();
        currentPos = pathToSmooth.getFirst();
        smoothedPath.add(currentPos);
        finalTarget = pathToSmooth.getLast();

        // Step-by-step smoothed path construction
        while ( !currentPos.equals(finalTarget) )
        {
            // If we are very close to the target, we set the current
            // position equal to the target and exit the loop
            if (euclidean(currentPos, finalTarget) <= granularity)
            {
                currentPos = finalTarget;
                smoothedPath.add(currentPos);
                break;
            }

            // Get the horizon point
            Vec2f horizon = getHorizon(pathToSmooth);

            // Get the next position making a step as big as the granularity
            // in the direction of the horizon
            // NOTE: Only one intersection is always possible here
            currentPos = getCircleLineIntersection(currentPos, horizon, currentPos, granularity).get(0);
            smoothedPath.add(currentPos);

        }

        return smoothedPath;
    }


    /* Return the current target position in a smooth space seen by the robot */
    public Vec2f getHorizon (LinkedList<Vec2f> path)
    {
        // Close to target
        if (euclidean(currentPos, finalTarget) <= lookAhead)
            return finalTarget;

        List<Vec2f> horizonOptions;
        Iterator<Vec2f> startIter = path.descendingIterator();
        Iterator<Vec2f> endIter = path.descendingIterator();
        startIter.next();

        while (startIter.hasNext())
        {
            // Next segment is considered
            Vec2f startNode = startIter.next();
            Vec2f endNode = endIter.next();
            horizonOptions = getCircleLineIntersection(startNode, endNode, currentPos, lookAhead);

            // Only one intersection
            if (horizonOptions.size() == 1)
                return horizonOptions.get(0);
            // If two intersection the closest to the end of the segment is considered
            else if (horizonOptions.size() == 2)
            {
                if (euclidean(horizonOptions.get(0), endNode) < euclidean(horizonOptions.get(1), endNode))
                    return horizonOptions.get(0);
                return horizonOptions.get(1);
            }
        }

        return null;
    }



    /*
        Given a segment (line) and a circle in a 2-dimensional space,
        this method returns the intersection point.
    */
    public static List<Vec2f> getCircleLineIntersection (Vec2f lineStart, Vec2f lineEnd, Vec2f center, float radius)
    {
        float baX = lineEnd.x - lineStart.x;
        float baY = lineEnd.y - lineStart.y;
        float caX = center.x - lineStart.x;
        float caY = center.y - lineStart.y;

        float a = baX * baX + baY * baY;
        float bBy2 = baX * caX + baY * caY;
        float c = caX * caX + caY * caY - radius * radius;

        float pBy2 = bBy2 / a;
        float q = c / a;

        float disc = pBy2 * pBy2 - q;
        if (disc < 0) return Collections.emptyList();

        float tmpSqrt = (float) Math.sqrt(disc);
        float abScalingFactor1 = -pBy2 + tmpSqrt;
        float abScalingFactor2 = -pBy2 - tmpSqrt;

        float maxX = Math.max(lineStart.x, lineEnd.x);
        float minX = Math.min(lineStart.x, lineEnd.x);
        float maxY = Math.max(lineStart.y, lineEnd.y);
        float minY = Math.min(lineStart.y, lineEnd.y);

        List<Vec2f> result = new ArrayList<>(2);
        Vec2f p1 = new Vec2f(lineStart.x - baX * abScalingFactor1, lineStart.y - baY * abScalingFactor1);
        if (minX <= p1.x && p1.x <= maxX && minY <= p1.y && p1.y <= maxY)
            result.add(p1);

        Vec2f p2 = new Vec2f( lineStart.x - baX * abScalingFactor2, lineStart.y - baY * abScalingFactor2);
        if (minX <= p2.x && p2.x <= maxX && minY <= p2.y && p2.y <= maxY)
            result.add(p2);

        return result;
    }


    /* The euclidean distance between two coordinates */
    public static float euclidean (Vec2f v1, Vec2f v2)
    {
        if (v1.equals(v2))
            return 0.0f;
        return (float) Math.sqrt(Math.pow(v1.x - v2.x, 2) + Math.pow(v1.y - v2.y, 2));
    }

}
