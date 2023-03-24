package sx.pathfind;

import sx.Vec2;



public class AStarHeapNode implements Comparable<AStarHeapNode>
{
    public float value;
    public Vec2 position;

    public AStarHeapNode(float value, Vec2 position)
    {
        this.value = value;
        this.position = position;
    }

    @Override
    public int compareTo(AStarHeapNode other)
    {
        return this.value < other.value ? -1 : 1;
    }

    @Override
    public boolean equals (Object other)
    {
        // !NOTE!: The check is made only on the position (not on k1 and k2)
        // because the same node cannot stay into the heap even with different
        // associated costs.

        if (other == null || other.getClass() != this.getClass())
            return false;
        AStarHeapNode otherNode = (AStarHeapNode) other;
        return this.position.equals(otherNode.position);
    }

    @Override
    public int hashCode ()
    {
        // !NOTE!: The check is made only on the position (not on k1 and k2)
        // because the same node cannot stay into the heap even with different
        // associated costs.
        return position.hashCode();
    }

}
