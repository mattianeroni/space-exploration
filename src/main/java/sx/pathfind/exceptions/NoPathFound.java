package sx.pathfind.exceptions;

public class NoPathFound extends Exception
{
    /* Exception raised when the path finding algorithm is not able to find a path */
    public NoPathFound (String message)
   {
       super(message);
   }
}
