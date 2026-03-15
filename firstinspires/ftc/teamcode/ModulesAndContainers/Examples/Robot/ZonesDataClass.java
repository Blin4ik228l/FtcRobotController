package org.firstinspires.ftc.teamcode.MainParts.Examples.Robot;

import static java.util.Collections.max;
import static java.util.Collections.min;

import androidx.annotation.NonNull;
import androidx.annotation.Nullable;

import org.firstinspires.ftc.teamcode.MainParts.Examples.Robot.RobotParts.DriveTrain.DrivetrainParts.Odometry.Parts.MathUtils.Vector2;

import java.util.ArrayList;
import java.util.Collection;
import java.util.Comparator;
import java.util.Deque;
import java.util.Iterator;
import java.util.List;
import java.util.Queue;

public abstract class ZonesDataClass {
    public abstract boolean isDotInIt(Dot dot);

    public abstract Dot getNearestDot(Dot from);
    private static final double EPSILON = 1e-6;  // FTC точность ~0.1мм
    public static class Dot{
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
    public class RectangleZone extends ZonesDataClass{
        //Ближе к 0
        public Dot head1;
        public Dot head2;
        public RectangleZone(Dot head1, Dot head2){
            if(head1.toVector().length() < head2.toVector().length()){
                this.head1 = head1;
                this.head2 = head2;
            }else {
                this.head1 = head2;
                this.head2 = head1;
            }

        }
        public boolean isDotInIt(Dot dot){
            return (head1.x <= dot.x  && dot.x <= head2.x) && (head1.y <= dot.y  && dot.y <= head2.y);
        }

        public Dot getNearestDot(Dot from){
            double x = Math.max(head1.x, Math.min(head2.x, from.x));
            double y = Math.max(head1.y, Math.min(head2.y, from.y));
            return new Dot(x, y);
        }
    }
    public class RoundZone extends ZonesDataClass{
        public Dot centr;
        public double radius;
        public RoundZone(Dot centr, double radius){
            this.centr = centr;
            this.radius = radius;
        }
        public double getArea(Dot head1, Dot head2, Dot head3){
            return Math.abs(head1.x * (head2.y - head3.y) +
                    head2.x * (head3.y - head1.y) +
                    head3.x * (head1.y - head2.y)) / 2.0;
        }
        public boolean isDotInIt(Dot dot){
            return Math.pow(dot.x - centr.x, 2)  - Math.pow(dot.y - centr.y, 2) - Math.pow(radius, 2) <= EPSILON;
        }

        public Dot getNearestDot(Dot from){
            if (centr.isEquals(from)) return new Dot(centr.x + radius, centr.y);
            else {
                Vector2 centrV = centr.toVector();
                Vector2 fromV = from.toVector();

                fromV.minus(centrV);
                fromV.normalize();
                fromV.multyplie(radius);
                centrV.add(fromV);

                return centrV.toDot();
            }
        }
    }
    public class TriangleZone extends ZonesDataClass{
        public Dot head1;
        public Dot head2;
        public Dot head3;
        public TriangleZone(Dot head1, Dot head2, Dot head3){
            this.head1 = head1;
            this.head2 = head2;
            this.head3 = head3;
        }
        public Dot closestPointOnSegment(Dot from, Dot head1, Dot head2) {
            Vector2 fromV = from.toVector();
            Vector2 head1V = head1.toVector();
            Vector2 head2V = head2.toVector();

            //Векторы от вершины
            head1V.minus(head2V); // head1 - head2
            fromV.minus(head2V);// fromV - head2

            double t = Vector2.scalarMultyplie(head1V, fromV) / Vector2.scalarMultyplie(head1V, head1V);  // t = (AP·AB) / |AB|²

            if (t <= 0) return head2;           // перед A
            if (t >= 1) return head1;           // после B

            // на отрезке: A + t·AB
            head1V.multyplie(t);
            head2V.add(head1V);
            return head2V.toDot();
        }

        public double getArea(Dot head1, Dot head2, Dot head3){
            return Math.abs(head1.x * (head2.y - head3.y) +
                            head2.x * (head3.y - head1.y) +
                            head3.x * (head1.y - head2.y)) / 2.0;
        }
        public Dot getNearestDot(Dot from){
            Dot[] heads = {
                    closestPointOnSegment(from, head1, head2),
                    closestPointOnSegment(from, head1, head3),
                    closestPointOnSegment(from, head2, head3)
            };

            Dot best = new Dot(0, 0);
            // Ищем минимум
            double bestDist = 1000000000;

            for (int i = 1; i < 3; i++) {
                Dot dot = heads[i];
                Vector2 dotV = dot.toVector();
                Vector2 fromV = from.toVector();

                dotV.minus(fromV);
                double dist = dotV.length();
                if (dist < bestDist) {
                    bestDist = dist;
                    best = heads[i];
                }
            }

            return best;
        }
        public boolean isDotInIt(Dot dot){
            double fullArea = getArea(head1, head2, head3);

            double area1 = getArea(dot, head2, head3);
            double area2 = getArea(dot, head1, head3);
            double area3 = getArea(dot, head1, head2);

            return (fullArea - (area1 + area2 + area3)) < EPSILON;
        }
    }
}
