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

   public static double[] motion_profile(double maxAccel, double maxVel, double distance, double elapsedTime) {
        // time to get to max velocity
        double acceleration_dt = maxVel / maxAccel;

        // if can't get to max, go as much as possible
        double halfway_distance = distance / 2;
        double acceleration_distance = 0.5 * maxAccel * Math.pow(acceleration_dt, 2);

        if (acceleration_distance > halfway_distance) {
            acceleration_dt = Math.sqrt(halfway_distance / (0.5 * maxAccel));
        }

        acceleration_distance = 0.5 * maxAccel * Math.pow(acceleration_dt, 2);

        // recalculate max velocity based on time to accelerate and decelerate
        maxVel = maxAccel * acceleration_dt;

        // decelerate at the same rate we accelerate
        double deceleration_dt = acceleration_dt;

        // calculate the time at max velocity
        double cruise_distance = distance - 2 * acceleration_distance;
        double cruise_dt = cruise_distance / maxVel;
        double deceleration_time = acceleration_dt + cruise_dt;

        // check if still in the motion profile
        double entire_dt = acceleration_dt + cruise_dt + deceleration_dt;
        if (elapsedTime > entire_dt) {
            return new double[] {distance, 0, 0};
        }

        // if we're accelerating
        if (elapsedTime < acceleration_dt) {
            // use the kinematic equation for acceleration
            return new double[] {0.5 * maxAccel * Math.pow(elapsedTime, 2), maxAccel * elapsedTime, maxAccel};
        }

        // if we're cruising (constant acceleration)
        else if (elapsedTime < deceleration_time) {
            acceleration_distance = 0.5 * maxAccel * Math.pow(acceleration_dt, 2);
            double cruise_current_dt = elapsedTime - acceleration_dt;

            // use the kinematic equation for constant velocity
            return new double[] {acceleration_distance + maxVel * cruise_current_dt,
                    maxVel,
                    0};
        }

        // if we're decelerating
        else {
            acceleration_distance = 0.5 * maxAccel * Math.pow(acceleration_dt, 2);
            cruise_distance = maxVel * cruise_dt;
            deceleration_time = elapsedTime - deceleration_time;

            // use the kinematic equations to calculate the instantaneous desired position
            return new double[] {acceleration_distance + cruise_distance + maxVel * deceleration_time - 0.5 * maxAccel * Math.pow(deceleration_time, 2),
                    maxVel - maxAccel * deceleration_time,
                    -maxAccel};
        }
    }
}
