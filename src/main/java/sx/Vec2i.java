package sx;


import java.util.Objects;



public class Vec2i
{

    /* An instance of this class represents a point in a 2-dimensional space */

    public int x = 0, y = 0;       // x and y coordinates


    public Vec2i(int x, int y)
    {
        this.x = x; this.y = y;
    }

    @Override
    public boolean equals(Object other)
    {
        if (other == null || other.getClass() != this.getClass())
            return false;
        Vec2i otherVec = (Vec2i) other;
        return this.x == otherVec.x && this.y == otherVec.y;
    }

    public int getX ()
    {
        return x;
    }

    public int getY()
    {
        return y;
    }


    @Override
    public int hashCode ()
    {
        return Objects.hash(x, y);
    }


    public String repr()
    {
        return "(" + x + ", " + y + ")";
    }


    public Vec2i copy()
    {
        return new Vec2i(x, y);
    }

}
