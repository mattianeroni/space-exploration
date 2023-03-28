package sx;


public class Coordinate
{

    /* An instance of this class represents a point in a 2-dimensional space */

    public float x = 0.0f, y = 0.0f;       // x and y coordinates


    public Coordinate (float x, float y)
    {
        this.x = x; this.y = y;
    }

    public String repr ()
    {
        return String.format("(%.2f, %.2f)", x, y);
    }

    public Coordinate copy()
    {
        return new Coordinate(x, y);
    }

}
