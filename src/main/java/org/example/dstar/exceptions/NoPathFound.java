package org.example.dstar.exceptions;

public class NoPathFound extends Exception
{
    /* Exception raised when the D* Lite is not able to find a path */

   public NoPathFound (String message)
   {
       super(message);
   }
}
