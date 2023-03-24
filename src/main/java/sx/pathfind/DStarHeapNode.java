package sx.pathfind;

import sx.Vec2;


public class DStarHeapNode implements Comparable<DStarHeapNode>
{

    /* An instance of this class represent a node into the D* heap */

    public float k1, k2;
    public Vec2 position;

    public DStarHeapNode(float k1, float k2, Vec2 position) {
        this.k1 = k1;
        this.k2 = k2;
        this.position = position;
    }

    @Override
    public int compareTo(DStarHeapNode other)
    {
        if (this.k1 == other.k1) {
            if (this.k2 == other.k2)
                return 0;
            return this.k2 < other.k2 ? -1 : 1;
        }
        return this.k1 < other.k1 ? -1 : 1;
    }

    @Override
    public boolean equals (Object other)
    {
        // !NOTE!: The check is made only on the position (not on k1 and k2)
        // because the same node cannot stay into the heap even with different
        // associated costs.

        if (other == null || other.getClass() != this.getClass())
            return false;
        DStarHeapNode otherNode = (DStarHeapNode) other;
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

    public String repr ()
    {
        return "HeapNode(" + k1 + ", " + k2 + ", " + position.repr() + ")";
    }

}
