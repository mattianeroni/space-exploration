package sx.pathfind;

import sx.Vec2i;



public class AStarHeapNode implements Comparable<AStarHeapNode>
{

    /*
        An instance of this class represent a node of the priority queue used by
        the A* algorithm.
    */

    public float value;         // The distance (or the A* path length estimation)
    public int counter;         // A counter to keep in order nodes with same value
    public Vec2i position;       // The position stored into the heap
    public float cost;          // The actual cost from the robot position to <position>
    public Vec2i parent;         // The parent node the position is reached from

    public AStarHeapNode(float value, int counter, Vec2i position, float cost, Vec2i parent)
    {
        this.value = value;
        this.counter = counter;
        this.position = position;
        this.cost = cost;
        this.parent = parent;
    }


    public AStarHeapNode(float value, Vec2i position, float cost, Vec2i parent)
    {
        this.value = value;
        this.counter = 0;
        this.position = position;
        this.cost = cost;
        this.parent = parent;
    }


    @Override
    public int compareTo(AStarHeapNode other)
    {
        if (value == other.value) {
            if (counter == other.counter) return 0;
            return counter < other.counter ? -1 : 1;
        }
        return value < other.value ? -1 : 1;
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
        return position.equals(otherNode.position) && counter == otherNode.counter;
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
