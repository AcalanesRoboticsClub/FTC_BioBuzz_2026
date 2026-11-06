package org.firstinspires.ftc.teamcode.Shooter;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.HardwareMap;

@Config
public class Shooter { // will contain elevator, hood, and launcher
    public Hood hood;
    public Launcher launcher;
    public Elevator elevator;

    public enum HOOD_STATE {HIGH, LOW, INC, DEC};
    public static HOOD_STATE hood_state = HOOD_STATE.LOW;

    public enum LAUNCH_TYPE {FLOWER, SHOOT};
    public static LAUNCH_TYPE launchTypeToggle = LAUNCH_TYPE.SHOOT;

    public Shooter(HardwareMap hardwareMap) {
        hood = new Hood(hardwareMap);
        launcher = new Launcher(hardwareMap);
        elevator = new Elevator(hardwareMap);
    }

    public void initialize() {
        elevator.toggleTopPos();
    }

    public void toggleFlowerMode() {
        if (launchTypeToggle == LAUNCH_TYPE.FLOWER) {
            launchTypeToggle = LAUNCH_TYPE.SHOOT;
            launcher.toggleShootPower();
        }
        else {
            elevator.toggleFlowerPos();
            hood.hoodFlower();
            launcher.toggleFlowerPower();
        }
    }

    public boolean isFlowerMode() {
        if (launchTypeToggle == LAUNCH_TYPE.FLOWER) {
            return true;
        }
        return false;
    }

    public void changeHoodPos(HOOD_STATE state) {
        hood_state = state;
        if (hood_state == HOOD_STATE.HIGH) {
            hood.hoodShootHigh();
        }
        if (hood_state == HOOD_STATE.LOW) {
            hood.hoodShootLow();
        }
//            if (state == HOOD_STATE.MIDHIGH) {
////                shooter.hood.hoodShoot();
//                shooter.hood.hoodShootMidHigh();
//            }
//            if (state == HOOD_STATE.MIDLOW) {
////                shooter.hood.hoodFlower();
//                shooter.hood.hoodShootMidLow();
//            }
        if (hood_state == HOOD_STATE.INC) {
            hood.incPosition();
        }
        if (hood_state == HOOD_STATE.DEC) {
            hood.decPosition();
        }
    }

    public void updateShooter(double dt) {
        hood.updateHood();
        elevator.runElevator(dt);
    }
}
