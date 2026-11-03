package org.firstinspires.ftc.teamcode.Shooter;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.HardwareMap;

@Config
public class Shooter {
    public Hood hood;
    public Launcher launcher;

    public Shooter(HardwareMap hardwareMap) {
        hood = new Hood(hardwareMap);
        launcher = new Launcher(hardwareMap);
    }
}
