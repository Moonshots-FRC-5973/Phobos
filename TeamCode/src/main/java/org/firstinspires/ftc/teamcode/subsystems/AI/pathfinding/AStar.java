//package org.firstinspires.ftc.teamcode.subsystems.AI.pathfinding;
//
//import com.qualcomm.robotcore.util.ElapsedTime;
//
//import org.firstinspires.ftc.teamcode.v1.systems.driveSystem.DriveSystem;
//import org.firstinspires.ftc.teamcode.v1.systems.sensors.Gyro;
//import org.firstinspires.ftc.teamcode.v1.systems.sli.FieldArea;
//
//import java.util.ArrayList;
//
//public class AStar {
//    public static ArrayList<Step> compute(FieldArea location, FieldArea target, ArrayList<FieldArea> obstacles ) {
//        ArrayList<Step> steps = new ArrayList<Step>();
//
//        // Start by finding the direction to the Target Area.
//        double directionToTarget = Math.atan2(location.getCenter().getX(), location.getCenter().getY());
//        return findPath(directionToTarget, location, target, obstacles);
//    }
//
//    public static boolean runFullStepList(ArrayList<Step> steps, Gyro gyro, ElapsedTime timer, DriveSystem driveSystem) {
//        for(Step step : steps) {
//            if(!runStep(step, gyro, timer, driveSystem)) return false;
//        }
//        return true;
//    }
//
//    private static ArrayList<Step> findPath(double angleToTarget, FieldArea location, FieldArea target, ArrayList<FieldArea> obstacles) {
//        // Find if there is an obstacle in the way
//
//        // Find shortest path around object
//
//        // recurse last two steps until there are no more problems on the path
//        return new ArrayList<Step>();
//    }
//
//    public static boolean runStep(Step step, Gyro gyro, ElapsedTime timer, DriveSystem driveSystem) {
//        double startTime = timer.milliseconds();
//        while(timer.milliseconds() - startTime <= step.time) {
//            driveSystem.drive(step.forward, step.turn);
//        }
//        return true;
//    }
//}
