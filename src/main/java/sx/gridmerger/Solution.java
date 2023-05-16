package sx.gridmerger;


public class Solution implements Comparable<Solution>
{
    /*
        An instance of this class represents a solution to the merging problem,
        made by a Transform, and a distance --i.e., distance between the transformed
        grid map and the reference grid map.

        NOTE: No indication to the reference grid map or the transformed grid map is kept.
    */

    public Transform transform;
    public float distance;


    public Solution (Transform transform, float distance)
    {
        this.transform = transform;
        this.distance = distance;
    }


    @Override
    public int compareTo (Solution other)
    {
        if (this.distance == other.distance)
            return 0;
        return this.distance < other.distance ? -1 : 1;
    }


    @Override
    public boolean equals (Object other)
    {
        if (other == null || other.getClass() != this.getClass())
            return false;
        Solution otherSol = (Solution) other;
        return this.transform.equals(otherSol.transform);
    }


    @Override
    public int hashCode () { return transform.hashCode(); }

}
