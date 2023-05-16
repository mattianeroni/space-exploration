package sx.gridmerger;

import sx.Vec2i;

import java.util.Objects;


public class Transform
{
    /*
        A transform is both a rotation and a translation a grid map
        or a grid cell may be subject.
    */

    public Vec2i translation;       // The translation along x- and y-axis
    public float angle;             // The rotation in radians
    public Vec2i center;            // The rotation center


    public Transform (Vec2i translation, Vec2i center, float angle)
    {
        this.translation = translation;
        this.angle = angle;
        this.center = center;
    }


    public Transform (int translationX, int translationY, Vec2i center, float angle)
    {
        this.translation = new Vec2i(translationX, translationY);
        this.angle = angle;
        this.center = center;
    }


    @Override
    public boolean equals (Object other)
    {
        if (other == null || other.getClass() != this.getClass())
            return false;
        Transform otherTransform = (Transform) other;
        return this.center.equals(otherTransform.center)
                && this.translation.equals(otherTransform.translation)
                && this.angle == otherTransform.angle;
    }


    @Override
    public int hashCode ()
    {
        return Objects.hash(center.x, center.y, translation.x, translation.y, angle);
    }

}
