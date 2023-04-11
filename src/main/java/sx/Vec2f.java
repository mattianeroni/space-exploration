package sx;


import java.util.Objects;

public class Vec2f
{

    /* An instance of this class represents a point in a 2-dimensional space */

    public float x = 0.0f, y = 0.0f;       // x and y coordinates


    public Vec2f(float x, float y)
    {
        this.x = x; this.y = y;
    }

    // NOTE: It would be better to check if they are close with a certain tolerance
    @Override
    public boolean equals(Object other)
    {
        if (other == null || other.getClass() != this.getClass())
            return false;
        Vec2f otherVec = (Vec2f) other;
        return this.x == otherVec.x && this.y == otherVec.y;
    }

    @Override
    public int hashCode ()
    {
        return Objects.hash(x, y);
    }

    public String repr ()
    {
        return String.format("(%.2f, %.2f)", x, y);
    }

    public Vec2f copy()
    {
        return new Vec2f(x, y);
    }

    public float getX ()
    {
        return x;
    }

    public float getY()
    {
        return y;
    }

}
