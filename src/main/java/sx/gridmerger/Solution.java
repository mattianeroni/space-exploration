package sx.gridmerger;


public class Solution implements Comparable<Solution>
{
    /*
        An instance of this class represents a solution to the merging problem,
        made by a Transform, and a distance --i.e., distance between the transformed
        grid map and the reference grid map.

        NOTE: No indication to the reference grid map or the transformed grid map is kept.
    */

    public Transform transform;         // The transform that generated this solution
    public float distance;              // The distance between the transformed grid map and the reference grid map
                                        // used when the solution was generated
    public int agreement;               // Number of known positions in which the transformed and the reference grid
                                        // maps were equal to each other
    public int disagreement;            // Number of known positions in which the transformed and the reference grid
                                        // maps were different


    public Solution (Transform transform, float distance, int agreement, int disagreement)
    {
        this.transform = transform;
        this.distance = distance;
        this.agreement = agreement;
        this.disagreement = disagreement;
    }


    /* Compute the acceptance indicator according to current agreement and disagreement */
    public float acceptanceIndicator ()
    {
        if ( (agreement + disagreement) == 0)
            return 0.0f;
        // NOTE: In the paper it was
        // return 1.0f - ( (float)agreement / (float)(agreement + disagreement));
        return (float)agreement / (float)(agreement + disagreement);
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
