package org.firstinspires.ftc.teamcode.subsystems.ai.sli;

public class Triangle {
    private Vec2 point1, point2, point3;

    public void setPoint1(Vec2 point) {
        this.point1 = point;
    }
    public Vec2 getPoint1() {
        return point1;
    }
    public void setPoint2(Vec2 point) {
        this.point2 = point;
    }
    public Vec2 getPoint2() {
        return point2;
    }
    public void setPoint3(Vec2 point) {
        this.point3 = point;
    }
    public Vec2 getPoint3() {
        return point3;
    }

    public Triangle(Vec2 point1, Vec2 point2, Vec2 point3) {
        this.point1 = point1;
        this.point2 = point2;
        this.point3 = point3;
    }
    public boolean isOverlapping(Triangle tri) {
        if(isOverlapping(tri.getPoint1())) return true;
        if(isOverlapping(tri.getPoint2())) return true;
        if(isOverlapping(tri.getPoint3())) return true;

        return false;
    }

    public boolean isOverlapping(Vec2 point) {

        double theta1 = Math.atan2(point1.getX() - point2.getX(),
                point1.getY() - point2.getY()
        );
        double theta2 = Math.atan2(point3.getX() - point2.getX(),
                point3.getY() - point2.getY()
        );

        if(theta1 > theta2) {
            if (Math.atan2(point.getX() - point1.getX(),
                    point.getY() - point1.getY()
            ) > theta1) return true;
            if (Math.atan2(point.getX() - point1.getX(),
                    point.getY() - point1.getY()
            ) < theta2) return true;
        } else {
            if (Math.atan2(point.getX() - point1.getX(),
                    point.getY() - point1.getY()
            ) < theta1) return true;
            if (Math.atan2(point.getX() - point1.getX(),
                    point.getY() - point1.getY()
            ) > theta2) return true;
        }

        return false;
    }
}
