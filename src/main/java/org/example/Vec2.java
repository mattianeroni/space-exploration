package org.example;

public class Vec2 {

    /* An instance of this class represents a point in a 2-dimensional space */

    public int x;       // x coordinate
    public int y;       // y coordinate

    public Vec2() {
        this.x = 0; this.y = 0;
    }

    public Vec2 (int x, int y){
        this.x = x; this.y = y;
    }

    public boolean equals(Vec2 other) {
        return this.x == other.x && this.y == other.y;
    }

}
