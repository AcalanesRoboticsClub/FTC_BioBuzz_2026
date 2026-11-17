package org.firstinspires.ftc.teamcode.Shooter;

import java.util.Map;
import java.util.TreeMap;

public class ShotTree {
    private final TreeMap<Double, ShotParams> tree = new TreeMap<>();

    public void add(double robotDist, double hoodAngle, double elevatorHeight, double launcherVel) {
        tree.put(robotDist, new ShotParams(hoodAngle, elevatorHeight, launcherVel));
    }

    public ShotParams get(double robotDist) {
        if (tree.isEmpty()) {
            return new ShotParams(0, 0, 0);
        }

        if (tree.containsKey(robotDist)) {
            return tree.get(robotDist);
        }

        Map.Entry<Double, ShotParams> lower = tree.floorEntry(robotDist);
        Map.Entry<Double, ShotParams> upper = tree.ceilingEntry(robotDist);

        if (lower == null) { return upper.getValue(); }
        if (upper == null) { return lower.getValue(); }

        double t = (robotDist - lower.getKey()) / (upper.getKey() - lower.getKey());
        return lower.getValue().interpolate(upper.getValue(), t);
    }
}
