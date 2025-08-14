package org.firstinspires.ftc.teamcode.Modules.Players.Pl1.MecanumDriveTrain.MathUtils;

import androidx.annotation.NonNull;

public class Position {
    private double x;
    private double y;
    private double heading;

    public Position(double x ,double y, double heading){
        this.x = x;
        this.y = y;
        this.heading = Math.toRadians(heading);
    }

    public Position(Position position){
        this.x = position.x;
        this.y = position.y;
        this.heading = position.heading;
    }

    public Position(Vector2 vector2, double heading){
        this.x = vector2.x;
        this.y = vector2.y;
        this.heading = heading;
    }

    public Position(){
        this.x = 0;
        this.y = 0;
        this.heading = 0;
    }

    public Vector2 toVector(){
        return new Vector2(this.x, this.y);
    }

    public void setX(double x){
        this.x = x;
    }

    public void setY(double y){
        this.y = y;
    }

    public void setHeading(double heading){
        this.heading = heading;
    }

    public double getX(){
        return this.x;
    }

    public double getY(){
        return this.y;
    }

    public double getHeading(){
        // Если направление робота будет больше +-2pi радиан (+-360 градусов), то приравняется
        // к остатку от деления на 2pi (360)
        return this.heading % (2 * Math.PI);
    }

    public void add(@NonNull Vector2 vector2, double heading){
        this.x += vector2.x;
        this.y += vector2.y;
        this.heading += heading;
    }

    @NonNull
    public Position clone(){
        return new Position(this);
    }

}
