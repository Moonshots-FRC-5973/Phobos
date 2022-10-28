//package org.firstinspires.ftc.teamcode.subsystems.ai;
//
//import org.firstinspires.ftc.teamcode.v1.constants.ai.PositionMapConstants;
//import org.firstinspires.ftc.teamcode.v1.systems.driveSystem.DriveSystem;
//import org.firstinspires.ftc.teamcode.v1.systems.sensors.Gyro;
//import org.firstinspires.ftc.teamcode.v1.systems.sli.FieldArea;
//import org.firstinspires.ftc.teamcode.v1.systems.sli.Triangle;
//import org.firstinspires.ftc.teamcode.v1.systems.sli.Vec2;
//
//import java.util.ArrayList;
//
//public class PositionMap {
//    private ArrayList<FieldArea> fieldAreas = new ArrayList<FieldArea>();
//
//    public PositionMap(ArrayList<FieldArea> objects) {
//        fieldAreas = objects;
//    }
//
//    public boolean goToArea(DriveSystem driveSystem, Gyro gyro, FieldArea obj) {
//        Triangle tris[] = new Triangle[PositionMapConstants.POINTS_ON_CIRCLE];
//        double degreesPerStep = 360 / PositionMapConstants.POINTS_ON_CIRCLE;
//
//        for(int i = 0; i < PositionMapConstants.POINTS_ON_CIRCLE; i++) {
//            tris[i] = new Triangle(
//                    new Vec2(
//                            PositionMapConstants.MINIMUM_DISTANCE * Math.cos(i * degreesPerStep) + Gyro.getLocation().getX(),
//                            PositionMapConstants.MINIMUM_DISTANCE * Math.sin(i * degreesPerStep) + Gyro.getLocation().getY()
//                    ),
//                    new Vec2(
//                            PositionMapConstants.MINIMUM_DISTANCE * Math.cos((i + 1) * degreesPerStep) + Gyro.getLocation().getX(),
//                            PositionMapConstants.MINIMUM_DISTANCE * Math.sin((i + 1) * degreesPerStep) + Gyro.getLocation().getY()
//                    ),
//                    Gyro.getLocation()
//            );
//        }
//
//        FieldArea robot = new FieldArea("Robot", tris);
//
//
//
//        return false;
//    }
//
//    public boolean isOverlappingObject(FieldArea obj) {
//        return false;
//    }
//
//    public FieldArea getFieldObject(String objName) {
//        for(FieldArea obj : fieldAreas)
//            if(obj.getName().equals(objName))
//                return obj;
//        return null;
//    }
//}
