package org.firstinspires.ftc.teamcode.ModulesAndContainers.Examples.Robot.RobotParts.DriveTrain.Odometry;

import org.firstinspires.ftc.teamcode.ModulesAndContainers.Examples.Robot.RobotParts.DriveTrain.Odometry.Parts.MathUtils.Position2D;
import org.firstinspires.ftc.teamcode.ModulesAndContainers.Examples.Robot.RobotParts.DriveTrain.Odometry.Parts.MathUtils.Vector2;
import org.firstinspires.ftc.teamcode.ModulesAndContainers.Examples.Robot.RobotParts.DriveTrain.Reason;

import java.util.ArrayList;

public class OdometryData {
    private Position2D robotPosition;
    private Vector2 robotVelocity, robotAccel, rotatableVector;
    private double robotHeadVel, robotHeadAccel;
    private double desisionMarg;

    public OdometryData(){
        this.robotPosition = new Position2D();
        this.robotVelocity = new Vector2();
        this.robotAccel = new Vector2();
        this.rotatableVector = new Vector2();
        this.robotHeadVel = 0;
        this.robotHeadAccel = 0;
    }
    public OdometryData(Vector2 linVel, double headVel){
        this.robotPosition = new Position2D();
        this.robotVelocity = linVel.copy();
        this.robotAccel = new Vector2();
        this.rotatableVector = new Vector2();
        this.robotHeadVel = headVel;
        this.robotHeadAccel = 0;
    }

    public OdometryData(Position2D pos, Vector2 linVel, Vector2 linAccel, double headVel, double headAccel){
        this.robotPosition = pos.clone();
        this.robotVelocity = linVel.copy();
        this.robotAccel = linAccel.copy();
        this.robotHeadVel = headVel;
        this.robotHeadAccel = headAccel;
    }
    public OdometryData(Position2D pos, Vector2 linVel,double headVel){
        this.robotPosition = pos.clone();
        this.robotVelocity = linVel.copy();
        this.robotAccel = new Vector2();
        this.robotHeadVel = headVel;
        this.robotHeadAccel = 0;
    }
    public OdometryData(OdometryData other){
        this(other.robotPosition, other.robotVelocity, other.robotAccel, other.robotHeadVel, other.robotHeadAccel);
    }
    public void setDesisionMarg(double desisionMarg) {
        this.desisionMarg = desisionMarg;
    }
    public double getDesisionMarg() {
        return desisionMarg;
    }

    public void set(OdometryData odometryData){
        setRobotPosition(odometryData.getRobotPosition());
        setRobotVelocity(odometryData.getRobotVelocity());
        setRobotAccel(odometryData.getRobotAccel());
        setRobotHeadVel(odometryData.getRobotHeadVel());
        setRobotHeadAccel(odometryData.getRobotHeadAccel());
    }

    public Vector2 getRotatableVector() {
        return rotatableVector;
    }

    public void setRotatableVector(Vector2 rotatableVector) {
        this.rotatableVector = rotatableVector;
    }

    public void setRotatableVector(double x, double y) {
        this.rotatableVector.x = x;
        this.rotatableVector.y = y;
    }

    public Position2D getRobotPosition() {
        return robotPosition;
    }
    public Vector2 getRobotVelocity() {
        return robotVelocity;
    }
    public Vector2 getRobotAccel() {
        return robotAccel;
    }

    public void setRobotPosition(Position2D robotPosition) {
        this.robotPosition = robotPosition;
    }

    public double getRobotHeadAccel() {
        return robotHeadAccel;
    }

    public double getRobotHeadVel() {
        return robotHeadVel;
    }

    public OdometryData setRobotVelocity(Vector2 robotVelocity) {
        this.robotVelocity = robotVelocity;
        return this;
    }

    public OdometryData setRobotAccel(Vector2 robotAccel) {
        this.robotAccel = robotAccel;
        return this;
    }

    public OdometryData setRobotHeadVel(double robotHeadVel) {
        this.robotHeadVel = robotHeadVel;
        return this;
    }

    public OdometryData setRobotHeadAccel(double robotHeadAccel) {
        this.robotHeadAccel = robotHeadAccel;
        return this;
    }

    public OdometryData rotateVelocity(){
        this.robotVelocity.rotateToGlobal(this.robotPosition.getHeading());
        return this;
    }
    public OdometryData rotateAccel(){
        this.robotAccel.rotateToGlobal(this.robotPosition.getHeading());
        return this;
    }

    public static class DataBuilder{
        private ArrayList<OdometryData> pathBuilder;
        private ArrayList<Reason> reasons;
        public DataBuilder(){
            pathBuilder = new ArrayList<>();
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
            pathBuilder.remove(0);
            reasons.remove(0);
            return this;
        }
    }
}
