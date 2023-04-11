package sx.frontier;

import sx.Vec2i;
import sx.pathfind.AStarHeapNode;

public class FrontierHeapNode  implements Comparable<FrontierHeapNode>
{
    /*
        An instance of this class represents a position to explore in the
        Breath First Search used by the Wavefront Frontier Detector.
    */

    public int priority;
    public Vec2i position;

    public FrontierHeapNode(int priority, Vec2i position)
    {
        this.priority = priority;
        this.position = position;
    }


    @Override
    public int compareTo(FrontierHeapNode other)
    {
        if (priority == other.priority)
            return 0;
        return priority < other.priority ? -1 : 1;
    }

    @Override
    public boolean equals (Object other)
    {
        if (other == null || other.getClass() != this.getClass())
            return false;
        FrontierHeapNode otherNode = (FrontierHeapNode) other;
        return position.equals(otherNode.position);
    }

    @Override
    public int hashCode ()
    {
        return position.hashCode();
    }

}
