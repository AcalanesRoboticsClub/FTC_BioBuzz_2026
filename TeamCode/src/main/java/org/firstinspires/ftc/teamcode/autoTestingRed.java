package org.firstinspires.ftc.teamcode;

import com.pedropathing.api.PoseFactory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name = "r-1tip", group = "Autonomous")
public class autoTestingRed extends autoTesting {
    private final PoseFactory poseFactory = PoseFactory.degrees().mirrorX(70.25); // Mirror all the poses

    @Override
    public void storeAllianceColor() { // Store color for teleop
        HandoffPoseStorage.alliance_color = HandoffPoseStorage.ALLIANCE.RED;
    }
}