package org.firstinspires.ftc.teamcode.ModulesAndContainers.Examples.Robot;

import static java.util.Collections.max;
import static java.util.Collections.min;

import org.firstinspires.ftc.teamcode.ModulesAndContainers.Examples.Robot.RobotParts.Odometry.Parts.MathUtils.Dot;
import org.firstinspires.ftc.teamcode.ModulesAndContainers.Examples.Robot.RobotParts.Odometry.Parts.MathUtils.Vector2;
import org.firstinspires.ftc.teamcode.ModulesAndContainers.Examples.Players.PL0.TeamAliance;

import java.util.ArrayList;

public abstract class ZonesDataClass  {
    public abstract boolean isDotInIt(Dot dot);
    public abstract Dot getNearestDot(Dot from);
    protected static final double EPSILON = 1e-6;
    protected Dot zoneCentr;
    public TeamAliance teamAliance;
    public ZoneRole zoneRole;

    public void setInformation(TeamAliance teamAliance, ZoneRole zoneRole){
        this.teamAliance = teamAliance;
        this.zoneRole = zoneRole;
    }
    public Dot getZoneCentr() {
        return zoneCentr;
    }
    public enum ZoneRole {
        PARKING,
        FIRE
    }
    public static class Builder extends ZonesDataClass{
        private ArrayList<ZonesDataClass> zones;
        public Builder(){
            zones = new ArrayList<>();
        }

        public Builder createZone(TeamAliance teamAliance, ZoneRole zoneRole, Dot...dots){
            if(dots.length == 3){
                TriangleZone triangleZone = new ZonesDataClass.TriangleZone(dots[0], dots[1], dots[2]);
                triangleZone.setInformation(teamAliance, zoneRole);
                zones.add(triangleZone);
            }else if (dots.length == 2){
                RectangleZone rectangleZone = new ZonesDataClass.RectangleZone(dots[0], dots[1]);
                rectangleZone.setInformation(teamAliance, zoneRole);
                zones.add(rectangleZone);
            };
            return this;
        }
        public Builder createZone(TeamAliance teamAliance, ZoneRole zoneRole, Dot dot, double radius){
            RoundZone roundZone = new ZonesDataClass.RoundZone(dot, radius);
            roundZone.setInformation(teamAliance, zoneRole);
            zones.add(roundZone);

            return this;
        }
        public ZonesDataClass getParkingZone(){
            ZonesDataClass zonesDataClass = null;
            for (ZonesDataClass zone : zones) {
                if (zone.teamAliance == teamAliance && zone.zoneRole == ZoneRole.PARKING){
                    zonesDataClass = zone;
                }
            }
            return zonesDataClass;
        }
        @Override
        public boolean isDotInIt(Dot dot) {
            boolean inIt = false;

            for (ZonesDataClass zone : zones) {
                inIt |= zone.isDotInIt(dot);
            }
            return inIt;
        }

        @Override
        public Dot getNearestDot(Dot from) {
            Dot near = zones.get(0).getNearestDot(from);
            Vector2 nearV = near.toVector();

            for (ZonesDataClass zone : zones) {
                Vector2 result = zone.getNearestDot(from).toVector();
                if(result.length() < nearV.length()){
                    nearV = result;
                }
            }
            return nearV.toDot();
        }
    }

    protected static class RectangleZone extends ZonesDataClass{
        //Ближе к 0
        private Dot head1;
        private Dot head2;
        public RectangleZone(Dot head1, Dot head2){
            this.head1 = new Dot(Math.min(head1.x, head2.x), Math.min(head1.y, head2.y));
            this.head2 = new Dot(Math.max(head1.x, head2.x), Math.max(head1.y, head2.y));
            this.zoneCentr = new Dot((head1.x + head2.x)/2, (head1.y + head2.y)/2);
        }
        @Override
        public boolean isDotInIt(Dot dot){
            return (head1.x <= dot.x  && dot.x <= head2.x) && (head1.y <= dot.y  && dot.y <= head2.y);
        }

        @Override
        public Dot getNearestDot(Dot from){
            double x = Math.max(head1.x, Math.min(head2.x, from.x));
            double y = Math.max(head1.y, Math.min(head2.y, from.y));
            return new Dot(x, y);
        }
    }
    protected static class RoundZone extends ZonesDataClass{
        private Dot centr;
        private double radius;
        public RoundZone(Dot centr, double radius){
            this.centr = centr;
            this.radius = radius;
            this.zoneCentr = centr;
        }
        public boolean isDotInIt(Dot dot){
            return Math.pow(dot.x - centr.x, 2) + Math.pow(dot.y - centr.y, 2) - Math.pow(radius, 2) <= EPSILON;
        }

        public Dot getNearestDot(Dot from){
            if (centr.isEquals(from)) return new Dot(centr.x + radius, centr.y);
            else {
                Vector2 centrV = centr.toVector();
                Vector2 fromV = from.toVector();

                Vector2 fc = fromV.minus(centrV);
                fc.normalize();
                fc.multyplie(radius);

                fc.add(centrV);

                return fc.toDot();
            }
        }
    }
    protected static class TriangleZone extends ZonesDataClass{
        private Dot a;
        private Dot b;
        private Dot c;
        public TriangleZone(Dot a, Dot b, Dot c){
            this.a = a;
            this.b = b;
            this.c = c;

            this.zoneCentr = new Dot((a.x+b.x+c.x)/3, (a.y+b.y+c.y)/3);
        }
        private Dot closestPointOnSegment(Dot from, Dot a, Dot b) {
            Vector2 p = from.toVector();  // P
            Vector2 A = a.toVector();     // A
            Vector2 B = b.toVector();     // B

            Vector2 AB = B.minus(A);      // B - A
            Vector2 AP = p.minus(A);      // P - A

            double t = Vector2.scalarMultyplie(AP, AB) / Vector2.scalarMultyplie(AB, AB);  // t = (AP·AB) / |AB|²

            if (t <= 0) return a;         // перед A
            if (t >= 1) return b;         // после B

            // на отрезке: A + t·AB
            Vector2 result = A.add(AB.multyplie2(t));
            return result.toDot();
        }

        private double getArea(Dot head1, Dot head2, Dot head3){
            return Math.abs(head1.x * (head2.y - head3.y) +
                            head2.x * (head3.y - head1.y) +
                            head3.x * (head1.y - head2.y)) / 2.0;
        }
        @Override
        public Dot getNearestDot(Dot from){
            Dot[] heads = {
                    closestPointOnSegment(from, a, b),
                    closestPointOnSegment(from, a, c),
                    closestPointOnSegment(from, b, c)
            };

            Dot best = heads[0];
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
        @Override
        public boolean isDotInIt(Dot dot){
            double fullArea = getArea(a, b, c);

            double area1 = getArea(dot, b, c);
            double area2 = getArea(dot, a, c);
            double area3 = getArea(dot, a, b);

            return Math.abs(fullArea - (area1 + area2 + area3)) < EPSILON;
        }
    }
}
