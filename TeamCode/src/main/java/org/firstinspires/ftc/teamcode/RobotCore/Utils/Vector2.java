package org.firstinspires.ftc.teamcode.RobotCore.Utils;

// Класс для работы с матрицами
public class Vector2 {
    public double x;
    public double y;
    private double vectorLength;

    public Vector2(double x, double y) {
        this.x = x;
        this.y = y;
    }

    public Vector2(Vector2 vector) {
        this.x = vector.x;
        this.y = vector.y;
        this.vectorLength = vector.vectorLength;
    }

    public Vector2() {
        x = y = 0;
    }

    public static Vector2 add(Vector2 a, Vector2 b) {
        return new Vector2(a.x + b.x, a.y + b.y);
    }

    public double length() {
        if (x == 0 & y == 0) {
            vectorLength = 0;
        } else if (x == 0) {
            vectorLength = x;
        } else if (y == 0) {
            vectorLength = y;
        } else {
            vectorLength = Math.sqrt(Math.pow(x, 2) + Math.pow(y, 2));
        }

        return vectorLength;
    }

    public void setVectorLength(double newLength){
        double sin = x/vectorLength;
        double cos = y/vectorLength;

        x = sin * newLength;
        y = cos * newLength;

    }
    public void multyplie(double a) {
        x *= a;
        y *= a;
        vectorLength *= Math.abs(a);
    }

    public Vector2 multyplie2(double a) {
        return new Vector2(x * a, y * a);
    }

    public void divide(double a) {
        x /= a;
        y /= a;
        vectorLength /= a;
    }

    public Vector2 divideReturnable(double a) {
       return new Vector2(x/a, y/a);
    }

    public static double scalarMultyplie(Vector2 a,Vector2 b) {
        return (a.x*b.x + a.y*b.y);
    }

    public void normalize() {
        double mag = length();

        if (mag == 0) {
            return;
        }

        x /= mag;
        y /= mag;
        vectorLength = 1;
    }

    public Vector2 minus (Vector2 b){
        return new Vector2(x - b.x, y - b.y);
    }

    public Vector2 minusArgs (Vector2 b, double multiplie){
        return new Vector2(multiplie * (x - b.x), multiplie * (y - b.y));
    }

    public void add(Vector2 a) {
        x += a.x;
        y += a.y;
        vectorLength = 0;
    }

    public void sub(Vector2 a) {
        x -= a.x;
        y -= a.y;
        vectorLength = length();
    }

    public void rotate(double Rad) {
        double oldX = x;
        double oldY = y;
        x = oldX*Math.cos(Rad) - oldY*Math.sin(Rad);
        y = oldX*Math.sin(Rad) + oldY*Math.cos(Rad);
    }

    public static Vector2 rotate(Vector2 vector, double Rad) {
        double x = vector.x * Math.cos(Rad) - vector.y * Math.sin(Rad);
        double y = vector.x * Math.sin(Rad) + vector.y * Math.cos(Rad);
        return new Vector2(x, y);
    }

//    public void rotateAround(double angle, double x, double y) {
//        this.x -= x;
//        this.y -= y;
//        rotate(angle); //idk if is scuffed or not lmao -Kyle
//        this.x +=x;
//        this.y +=y;
//    }
//
//    public static Vector2 rotateAround(Vector2 vector, double angle, double x, double y) {
//        double vx = vector.x-x;
//        double vy = vector.y-y;
//        Vector2 temp = Vector2.rotate(new Vector2(vx,vy), angle);
//        return new Vector2(temp.x+x, temp.y+y);
//    }

}
