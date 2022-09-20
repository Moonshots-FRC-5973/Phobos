package org.firstinspires.ftc.teamcode.subsystems.sli;

import java.util.ArrayList;

public class FieldArea {
    String name;
    Triangle[] obj;
    Vec2 center;

    public FieldArea(String name, Triangle[] points) {
        this.name = name;
        obj = points;
        double x = 0;
        double y = 0;
        ArrayList<Vec2> vec2s = new ArrayList<>();

        for(Triangle tri : points) {
            if(!vec2s.contains(tri.getPoint1())) vec2s.add(tri.getPoint1());
            if(!vec2s.contains(tri.getPoint2())) vec2s.add(tri.getPoint2());
            if(!vec2s.contains(tri.getPoint3())) vec2s.add(tri.getPoint3());
        }

        for(Vec2 vec2 : vec2s) {
            x += vec2.getX();
            y += vec2.getY();
        }

        x /= vec2s.size();
        y /= vec2s.size();

        center = new Vec2(x, y);
    }

    public String getName() {
        return this.name;
    }

    public boolean isOverlapping(Triangle[] obj) {
        for(Triangle tri : obj) {
            for(Triangle triangle : this.obj) {
                if(tri.isOverlapping(triangle)) {
                    return true;
                }
            }
        }
        return false;
    }

    public boolean isOverlapping(Vec2 point) {
        for(Triangle tri : this.obj) {
            if(tri.isOverlapping(point)) {
                return true;
            }
        }
        return false;
    }

    public Vec2 getCenter() {
        return center;
    }
}
