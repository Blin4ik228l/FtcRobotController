package org.firstinspires.ftc.teamcode.MainParts.Examples.Robot.RobotParts.Odometry;

import org.firstinspires.ftc.teamcode.MainParts.Examples.Robot.RobotParts.Odometry.Parts.MathUtils.Position2D;
import org.firstinspires.ftc.teamcode.MainParts.Examples.Robot.RobotParts.Odometry.Parts.MathUtils.Vector2;
import org.firstinspires.ftc.teamcode.MainParts.Examples.Players.Enums.Reason;

import java.util.ArrayList;

public class OdometryData {
    private Position2D position;
    private Vector2 velocity, accel;
    private double headVel, headAccel;
    private double desisionMarg;

    public OdometryData(){
        this.position = new Position2D();
        this.velocity = new Vector2();
        this.accel = new Vector2();
        this.headVel = 0;
        this.headAccel = 0;
    }
    public OdometryData(Vector2 linVel, double headVel){
        this.position = new Position2D();
        this.velocity = linVel.copy();
        this.accel = new Vector2();
        this.headVel = headVel;
        this.headAccel = 0;
    }

    public OdometryData(Position2D pos, Vector2 linVel, Vector2 linAccel, double headVel, double headAccel){
        this.position = pos.clone();
        this.velocity = linVel.copy();
        this.accel = linAccel.copy();
        this.headVel = headVel;
        this.headAccel = headAccel;
    }
    public OdometryData(Position2D pos, Vector2 linVel, double headVel){
        this.position = pos.clone();
        this.velocity = linVel.copy();
        this.accel = new Vector2();
        this.headVel = headVel;
        this.headAccel = 0;
    }
    public OdometryData(OdometryData other){
        this(other.position, other.velocity, other.accel, other.headVel, other.headAccel);
    }
    public void setDesisionMarg(double desisionMarg) {
        this.desisionMarg = desisionMarg;
    }
    public double getDesisionMarg() {
        return desisionMarg;
    }

    public void set(OdometryData odometryData){
        setPosition(odometryData.getPosition());
        setVelocity(odometryData.getVelocity());
        setAccel(odometryData.getAccel());
        setHeadVel(odometryData.getHeadVel());
        setHeadAccel(odometryData.getHeadAccel());
    }
    public void add(OdometryData odometryData){
        getPosition().add(odometryData.getPosition().getX(), odometryData.getPosition().getY(), odometryData.getPosition().getHeading());
        setVelocity(odometryData.getVelocity());
        setAccel(odometryData.getAccel());
        setHeadVel(odometryData.getHeadVel());
        setHeadAccel(odometryData.getHeadAccel());
    }
    public void reset(){
        position.setX(0);
        position.setY(0);
        position.setHeading(0);
    }

    public Position2D getPosition() {
        return position;
    }
    public Vector2 getVelocity() {
        return velocity;
    }
    public Vector2 getAccel() {
        return accel;
    }

    public void setPosition(Position2D position) {
        this.position = position;
    }

    public double getHeadAccel() {
        return headAccel;
    }

    public double getHeadVel() {
        return headVel;
    }

    public OdometryData setVelocity(Vector2 velocity) {
        this.velocity = velocity;
        return this;
    }

    public OdometryData setAccel(Vector2 accel) {
        this.accel = accel;
        return this;
    }

    public OdometryData setHeadVel(double headVel) {
        this.headVel = headVel;
        return this;
    }

    public OdometryData setHeadAccel(double headAccel) {
        this.headAccel = headAccel;
        return this;
    }

    public OdometryData rotateVelocity(){
        this.velocity.rotateToGlobal(this.position.getHeading());
        return this;
    }
    public OdometryData rotateAccel(){
        this.accel.rotateToGlobal(this.position.getHeading());
        return this;
    }

    public static class DataBuilder{
        private ArrayList<OdometryData> pathBuilder;
        private ArrayList<Reason> reasons;

        public DataBuilder(){
            pathBuilder = new ArrayList<>();
            reasons = new ArrayList<>();
        }

        public DataBuilder createPath(OdometryData odometryData, Reason reason){
            pathBuilder.add(odometryData);
            reasons.add(reason);
            return this;
        }
        public DataBuilder createPath(double x, double y, double head, double linVel, double headVel, Reason reason){
            pathBuilder.add(new OdometryData(new Position2D(x, y ,head), new Vector2(linVel), Math.toRadians(headVel)));
            reasons.add(reason);
            return this;
        }

        public DataBuilder createPath(Position2D position2D, double linVel, double headVel, Reason reason){
            pathBuilder.add(new OdometryData(position2D, new Vector2(linVel), Math.toRadians(headVel)));
            reasons.add(reason);
            return this;
        }
        public OdometryData getData(){
            return pathBuilder.get(0);
        }
        public Reason getReason(){
            return reasons.get(0);
        }

        public DataBuilder switchPose(){
            if(pathBuilder.isEmpty()) return this;
            pathBuilder.remove(0);
            reasons.remove(0);
            return this;
        }
    }
}
