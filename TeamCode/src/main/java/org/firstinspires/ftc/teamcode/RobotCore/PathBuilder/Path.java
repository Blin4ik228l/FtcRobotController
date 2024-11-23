package org.firstinspires.ftc.teamcode.RobotCore.PathBuilder;

import org.firstinspires.ftc.teamcode.RobotCore.Utils.Vector2;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;

public class Path {

    private List<Vector2> rawPoints = new ArrayList<>();
    private List<Vector2> rawAndControlPoints = new ArrayList<>();

    private HashMap<Double, Vector2> pathPoints = new HashMap<>();

    enum Mode {
        START_TO_END,
        START_TO_START,
        END_TO_START,
        END_TO_END
    }

    public Path() {

    }

    public Path add(double x, double y) {
        rawPoints.add(new Vector2(x,y));
        return this;
    }

    public Path add(Vector2 vec) {
        rawPoints.add(new Vector2(vec));
        return this;
    }

    public Path buildPolyline() {
        return this;
    }

    public Path buildSpline() {

        return this;
    }
}
