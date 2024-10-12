package org.firstinspires.ftc.teamcode;

public class Position extends Matrix{

    Position(double x ,double y, double heading){
        super(3,1);
        this.Mat = new double[][]{{x, y, heading}};
    }
    Position(Position position){
        super(position);
    }
    Position(Matrix matrix, double heading){
        super(matrix);
    }
    Position(){
        super(3,1);
        this.Mat[0][0] = 0;
        this.Mat[1][0] = 0;
        this.Mat[2][0] = 0;
    }

    public Position turn(double rad){
        return (Position)super.turn(rad);
    }

    public void setX(double x){
        this.Mat[0][0] = x;
    }

    public void setY(double y){
        this.Mat[1][0] = y;
    }

    public void setHeading(double heading){
        this.Mat[2][0] = heading;
    }

    public double getX(){
        return this.Mat[0][0];
    }

    public double getY(){
        return this.Mat[1][0];
    }

    public double getHeading(){
        return this.Mat[2][0];
    }
}
