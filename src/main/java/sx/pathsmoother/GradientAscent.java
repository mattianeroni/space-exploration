package sx.pathsmoother;

import sx.Coordinate;
import sx.Vec2;

import java.util.ArrayList;
import java.util.LinkedList;
import java.util.List;
import java.util.stream.Collectors;


public class GradientAscent implements PathSmoother
{

    /*
        DEPRECATED: This doesn't work.

        This class contains an implementation of the Gradient Ascent algorithm to smooth a path.
        Please, note this is just a path smoother, not a trajectory planner considering inertia,
        physics components, and collisions detection.
    */

    public float alpha;                            // Weight given to the original path corresponding position
    public float beta;                             // Weight given to the close positions
    public double tolerance = 0.001;               // The tolerance required
    public int maxiter = 1000;                     // Maximum number of iterations used as stopping criteria



    public GradientAscent (float alpha, float beta)
    {
        this.alpha = alpha;
        this.beta = beta;
    }


    public GradientAscent (float alpha, float beta, double tolerance)
    {
        this.alpha = alpha;
        this.beta = beta;
        this.tolerance = tolerance;
    }


    public GradientAscent (float alpha, float beta, double tolerance, int maxiter)
    {
        this.alpha = alpha;
        this.beta = beta;
        this.tolerance = tolerance;
        this.maxiter = maxiter;
    }


    @Override
    public LinkedList<Coordinate> smooth (LinkedList<Coordinate> path)
    {
        // Copy original path converting the grid map notation to a 2-dimensional notation
        // (ArrayList is used instead of linked list for faster access via id)
        List<Coordinate> newPath = new ArrayList<>();
        for (Coordinate i : path)
            newPath.add(i.copy());

        // Smoothing iterative process
        double change = Double.POSITIVE_INFINITY;
        int counter = 0;

        while (change > tolerance && counter < maxiter)
        {
            change = 0.0;
            for (int i = 1; i < path.size() - 1; i++)
            {
                Coordinate position = newPath.get(i);
                Coordinate old_position = path.get(i);
                Coordinate prev = newPath.get(i - 1);
                Coordinate next = newPath.get(i + 1);

                position.x += alpha * ( old_position.x - position.x) + beta * (next.x - prev.x - (2 * position.x));
                position.y += alpha * ( old_position.y - position.y) + beta * (next.y - prev.y - (2 * position.y));

                change += euclidean(position, old_position);
            }

            counter++;
        }

        // Return smoothed path as linked list
        return newPath.stream().collect(Collectors.toCollection(LinkedList::new));
    }


    /* The euclidean distance between two coordinates */
    public static float euclidean (Coordinate v1, Coordinate v2)
    {
        if (v1.equals(v2))
            return 0.0f;
        return (float) Math.sqrt(Math.pow(v1.x - v2.x, 2) + Math.pow(v1.y - v2.y, 2));
    }


}
