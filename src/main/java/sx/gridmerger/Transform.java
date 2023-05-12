package sx.gridmerger;

import sx.Vec2i;


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

}
