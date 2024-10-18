package org.firstinspires.ftc.teamcode;

public class Point {
    int x;
    int y;

    public Point(int x, int y) {
        this.x = x;
        this.y = y;
    }

    public Point(char x_in, int y_in) {
        x = x_in;
        y = y_in;
    }

    public Point(int x_in) {
        x = x_in;
    }

    public Point(Point another) {
        this.x = another.x;
        this.y = another.y;
    }

    static void dooy() {
        System.out.println();
    }

}
