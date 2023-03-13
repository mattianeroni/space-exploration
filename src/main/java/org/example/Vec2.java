package org.example;

import java.util.Objects;

public class Vec2
{

    /* An instance of this class represents a point in a 2-dimensional space */

    public int x, y;       // x and y coordinates

    public Vec2()
    {
        this.x = 0; this.y = 0;
    }

    public Vec2 (int x, int y)
    {
        this.x = x; this.y = y;
    }

    @Override
    public boolean equals(Object other)
    {
        if (other == null || other.getClass() != this.getClass())
            return false;
        Vec2 otherVec = (Vec2) other;
        return this.x == otherVec.x && this.y == otherVec.y;
    }


    @Override
    public int hashCode ()
    {
        return Objects.hash(x, y);
    }

}
