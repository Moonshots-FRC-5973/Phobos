package org.firstinspires.ftc.teamcode.subsystems.sli;

public class Vec2 {
    private double x;
    private double y;

    public Vec2(double z, double w) {
        x = z;
        y = w;
    }

    public void setX(double x) {
        this.x = x;
    }
    public double getX() {
        return x;
    }
    public void setY(double y) {
        this.y = y;
    }
    public double getY() {
        return y;
    }
    public boolean equals(Vec2 obj) {
        return (obj.getX() == x) && (obj.getY() == y);
    }
}
