package sx.frontier;

import org.locationtech.jts.geom.Envelope;
import sx.Vec2i;

import javax.lang.model.type.IntersectionType;
import java.util.*;

public class Frontier
{

    /* An instance of this class represents a frontier */

    public Envelope box;           // The bounding box containing the frontier
    public List<Vec2i> cells;      // The set of cells that make part of the frontier
    public Vec2i barycenter;       // The frontier barycenter where the robot is supposed to go to visit it


    public Frontier ()
    {
        this.cells = new ArrayList<>();
        this.barycenter = null;
        this.box = null;
    }


    public Frontier (Set<Vec2i> cellsSet)
    {
        this.cells = new ArrayList<>();
        cells.addAll(cellsSet);

        // NOTE: According to literature, it is better to compute the barycenter
        // using the median than using the mean
        Collections.sort(cells, Comparator.comparing(Vec2i::getX));
        int xBarycenter = cells.get((int) cells.size() / 2).x;
        Collections.sort(cells, Comparator.comparing(Vec2i::getY));
        int yBarycenter = cells.get((int) cells.size() / 2).y;
        this.barycenter = new Vec2i(xBarycenter, yBarycenter);

        int minX = Integer.MAX_VALUE, minY = Integer.MAX_VALUE;
        int maxX = Integer.MIN_VALUE, maxY = Integer.MIN_VALUE;
        for (Vec2i cell : cells)
        {
            minX = Math.min(minX, cell.x);
            minY = Math.min(minY, cell.y);
            maxX = Math.max(maxX, cell.x);
            maxY = Math.max(maxY, cell.y);
        }
        this.box = new Envelope(minX, maxX, minY, maxY);

    }

}
