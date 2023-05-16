package sx.gridmerger;

import java.util.TreeSet;


public class LimitedSizeSet<T extends Comparable<T>>
{
    /*
        An instance of this class represents a limited size set where
        only the Nth best elements are kept (where N is the capacity of the
        data structure).

        NOTE: The stored elements must be comparable, and the 'best' are the
        'lowest' ones.

        NOTE: This datastructure is not thread-safe.
    */

    private int maxSize;            // The maximum number of elements that can be stored
    public TreeSet<T> elements;     // The store elements sorted from the lowest to the highest


    public LimitedSizeSet (int maxSize)
    {
        this.maxSize = maxSize;
        this.elements = new TreeSet<>();
    }

    /* effective number of elements in the LimitedSizeSet */
    public int size ()
    {
        return elements.size();
    }


    /* Check if an element would be stored into the LimitedSizeSet */
    public boolean wouldKeep (T i)
    {
        if (elements.size() < maxSize || i.compareTo(elements.last()) < 0)
            return true;
        return false;
    }

    /* Add an element to the LimitedSizeSet (only if possible) */
    public void add (T i)
    {
        if (wouldKeep(i))
        {
            if (elements.size() == maxSize)
                elements.pollLast();
            elements.add(i);
        }
    }


}
