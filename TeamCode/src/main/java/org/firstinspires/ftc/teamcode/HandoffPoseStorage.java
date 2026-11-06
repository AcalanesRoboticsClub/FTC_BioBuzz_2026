package org.firstinspires.ftc.teamcode;

import com.pedropathing.math.Pose;

public class HandoffPoseStorage {
    public enum ALLIANCE {RED, BLUE};
    public static ALLIANCE alliance_color;

    public static Pose robotPose;

    public HandoffPoseStorage() {
        robotPose = Robot.robotPose;
    }
}
