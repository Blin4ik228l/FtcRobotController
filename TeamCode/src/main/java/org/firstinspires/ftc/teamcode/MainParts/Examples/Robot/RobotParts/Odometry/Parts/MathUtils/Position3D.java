package org.firstinspires.ftc.teamcode.MainParts.Examples.Robot.RobotParts.Odometry.Parts.MathUtils;

public class Position3D {
    private double x, y, z;
    private double roll, pitch, yaw;
    public Position3D(double x, double y, double z, double roll, double pitch, double yaw){
        this.x = x;
        this.y = y;
        this.z = z;
        this.roll = roll;
        this.pitch = pitch;
        this.yaw = yaw;
    }
    public Position3D(){
        this.x = 0;
        this.y = 0;
        this.z = 0;
        this.roll = 0;
        this.pitch = 0;
        this.yaw = 0;
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

    public double getRoll() {
        return roll;
    }

    public double getPitch() {
        return pitch;
    }

    public double getYaw() {
        return yaw;
    }

    public void setX(double x) {
        this.x = x;
    }

    public void setY(double y) {
        this.y = y;
    }

    public void setZ(double z) {
        this.z = z;
    }

    public void setRoll(double roll) {
        this.roll = roll;
    }

    public void setPitch(double pitch) {
        this.pitch = pitch;
    }

    public void setYaw(double yaw) {
        this.yaw = yaw;
    }
}
