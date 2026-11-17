package org.firstinspires.ftc.teamcode.Shooter;


import com.pedropathing.math.Pose;


public class ShotParams {
    public final double hoodAngle;
    public final double elevatorHeight;
    public final double launchVel;


    public ShotParams(double hoodAngle, double elevatorHeight, double launchVel) {
        this.hoodAngle = hoodAngle;
        this.elevatorHeight = elevatorHeight;
        this.launchVel = launchVel;
    }


    public ShotParams interpolate(ShotParams next, double t) {
        double shiftHoodAngle = this.hoodAngle + (next.hoodAngle - this.hoodAngle) * t;
        double shiftelevatorHeight = this.elevatorHeight + (next.elevatorHeight - this.elevatorHeight) * t;
        double shiftLaunchVel = this.launchVel + (next.launchVel - this.launchVel) * t;


        return new ShotParams(shiftHoodAngle, shiftelevatorHeight, shiftLaunchVel);
    }


}
