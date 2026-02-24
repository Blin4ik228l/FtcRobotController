package org.firstinspires.TaskAndArgs;

import org.firstinspires.ftc.teamcode.ModulesAndContainers.Examples.Robot.RobotParts.Odometry.Parts.MathUtils.Position2D;

public class Args {
    public static class DriveArgs extends Args{
        public DriveArgs(Position2D targetPosition2D, double targetSpeed){
            position2D = targetPosition2D;
            speed = targetSpeed;
        }
        public Position2D position2D;
        public double speed;
    }
    public static class LiftArgs extends Args{
        public LiftArgs(double targetHeight, double targetPower){
            height = targetHeight;
            power = targetPower;
        }
        public double height;
        public double power;
    }

    public static class ServoArgs extends Args{
        public ServoArgs(String targetServoName, double targetServoPos){
            servoName = targetServoName;
            servoPos = targetServoPos;
        }
        public String servoName;
        public double servoPos;
    }

}
