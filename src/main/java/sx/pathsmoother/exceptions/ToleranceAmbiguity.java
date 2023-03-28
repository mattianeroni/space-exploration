package sx.pathsmoother.exceptions;

public class ToleranceAmbiguity extends Exception
{
    /*
        Exception raised when the smoothed path cannot reach the required
        level of tolerance.
    */

    public ToleranceAmbiguity (String message)
    {
        super(message);
    }
}
