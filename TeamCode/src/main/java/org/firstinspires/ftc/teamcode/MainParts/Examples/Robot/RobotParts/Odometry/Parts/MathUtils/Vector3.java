package org.firstinspires.ftc.teamcode.MainParts.Examples.Robot.RobotParts.Odometry.Parts.MathUtils;

public class Vector3 {
    private double x, y, z;
    public Vector3(double x, double y ,double z){
        this.x = x;
        this.y = y;
        this.z = z;
    }

    public double getX() {
        return x;
    }

    public double getY() {
        return y;
    }

    public double getZ() {
        return z;
    }

    public Vector3 rotateX(double roll){
        double cos = Math.cos(roll);
        double sin = Math.sin(roll);
        return new Vector3(
                x,
                y * cos - z * sin,
                y * sin + z * cos
        );
    }
    public Vector3 rotateY(double pitch){
        double cos = Math.cos(pitch);
        double sin = Math.sin(pitch);
        return new Vector3(
                x * cos - y * sin,
                y,
                -x * sin + z * cos
        );
    }

    public Vector3 rotateZ(double yaw){
        double cos = Math.cos(yaw);
        double sin = Math.sin(yaw);
        return new Vector3(
                x * cos - y * sin,
                x * sin + y * cos,
                z
        );
    }

    public Vector3 rotate(double roll, double pitch, double yaw){
        return this.rotateX(roll).rotateY(pitch).rotateZ(yaw);
    }
}
