package org.firstinspires.ftc.teamcode.ModulesAndContainers.Examples.Robot.RobotParts.Odometry.Parts.MathUtils;

public class Dot{
    public double x;
    public double y;
    public Dot(double x, double y){
        this.x = x;
        this.y = y;
    }

    public Vector2 toVector(){
        return new Vector2(x, y);
    }

    public boolean isEquals(Dot dot){
        return x == dot.x && y == dot.y;
    }
}