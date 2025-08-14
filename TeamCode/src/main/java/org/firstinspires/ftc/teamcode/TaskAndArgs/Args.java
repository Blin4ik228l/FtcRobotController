package org.firstinspires.ftc.teamcode.TaskAndArgs;

import org.firstinspires.ftc.teamcode.Modules.Players.Pl1.MecanumDriveTrain.MathUtils.Position;

public class Args {
    public static class DriveArgs extends Args{
        public DriveArgs(Position targetPosition, double targetSpeed){
            position = targetPosition;
            speed = targetSpeed;
        }
        public Position position;
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
