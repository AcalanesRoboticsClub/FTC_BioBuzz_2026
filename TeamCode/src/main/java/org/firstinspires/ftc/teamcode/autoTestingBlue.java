package org.firstinspires.ftc.teamcode;

import com.pedropathing.api.PoseFactory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.HandoffPoseStorage;

@Autonomous(name = "b-1tip", group = "Autonomous")
public class autoTestingBlue extends autoTesting {

    @Override
    public void storeAllianceColor() { // Store color for teleop
        HandoffPoseStorage.alliance_color = HandoffPoseStorage.ALLIANCE.BLUE;
    }
}