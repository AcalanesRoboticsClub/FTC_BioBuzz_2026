package org.firstinspires.ftc.teamcode.Shooter;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.HardwareMap;

/*
 * Shooter class holds elevator, hood, and paddle mechanisms
 */

@Config
public class Shooter {
    public Hood hood;
    public Paddle paddle;
    public Elevator elevator;

    public enum HOOD_STATE {HIGH, LOW, INC, DEC, INC_SMALL, DEC_SMALL};
    public static HOOD_STATE hood_state = HOOD_STATE.LOW;

    public enum PADDLE_TYPE {FLOWER, SHOOT};
    public static PADDLE_TYPE paddleTypeToggle = PADDLE_TYPE.SHOOT;

    double timeToElevatorDown = 10;

    public Shooter(HardwareMap hardwareMap) {
        hood = new Hood(hardwareMap);
        paddle = new Paddle(hardwareMap);
        elevator = new Elevator(hardwareMap);
    }

    /**
     * At the start of the match, moves elevator slightly upward
     */
    public void deploy() {
        elevator.goToPos(3);
    }

    /**
     * Toggles flower mode
     * On - paddle changes to flower velocity, hood moves to top position, elevator moves to flower height
     * Off - paddle changes to shoot velocity
     */
    public void toggleFlowerMode() {
        if (paddleTypeToggle == PADDLE_TYPE.FLOWER) {
            paddleTypeToggle = PADDLE_TYPE.SHOOT;
            paddle.toggleShootVelocity();
        }
        else {
            paddleTypeToggle = PADDLE_TYPE.FLOWER;
            elevator.toggleFlowerPos();
            hood.hoodFlower();
            paddle.toggleFlowerVelocity();
        }
    }

    /**
     * Checks if flower mode is on
     * @return true if flower mode is on
     */
    public boolean isFlowerMode() {
        if (paddleTypeToggle == PADDLE_TYPE.FLOWER) {
            return true;
        }
        return false;
    }

    /**
     * Changes the hood position based on the inputted state
     * @param state the state to change the hood to
     */
    public void changeHoodPos(HOOD_STATE state) {
        hood_state = state;
        if (hood_state == HOOD_STATE.HIGH) { // top position
            hood.hoodShootHigh();
        }
        if (hood_state == HOOD_STATE.LOW) { // bottom position
            hood.hoodShootLow();
        }
        if (hood_state == HOOD_STATE.INC_SMALL) { // increments by a small amount
            hood.incPosition_small();
        }
        if (hood_state == HOOD_STATE.DEC_SMALL) { // decrements by a small amount
            hood.decPosition_small();
        }
//            if (state == HOOD_STATE.MIDHIGH) {
////                shooter.hood.hoodShoot();
//                shooter.hood.hoodShootMidHigh();
//            }
//            if (state == HOOD_STATE.MIDLOW) {
////                shooter.hood.hoodFlower();
//                shooter.hood.hoodShootMidLow();
//            }
        if (hood_state == HOOD_STATE.INC) { // increments by a large amount
            hood.incPosition();
        }
        if (hood_state == HOOD_STATE.DEC) { // decrements by a large amount
            hood.decPosition();
        }
    }

    /**
     * Sets the paddle, hood, and elevator to shooting positions/velocities for one pollen (defualt)
     */
    public void setShootProfile() {
        paddle.setShootVelocity(2000);
        hood.setPos(0.22);
        elevator.goToPos(0);
    }

    /**
     * Calculates position/velocity of paddle, hood, and elevator and sets them
     * @param numPollen number of pollen in sleeve
     * @param numNectar number of nectar in sleeve
     */
    public void setShootProfile(double numPollen, double numNectar) {
        paddle.setShootVelocity(2000);
        hood.setPos(0.22);
        elevator.goToPos(0);
    }

    /**
     * Calculates position/velocity of paddle, hood, and elevator and sets them
     * @param numPollen number of pollen in sleeve
     * @param numNectar number of nectar in sleeve
     * @param pose robot pose
     */
    public void setShootProfile(double numPollen, double numNectar, double pose) {
        paddle.setShootVelocity(2000);
        hood.setPos(0.22);
        elevator.goToPos(0);
    }

    /**
     * Updates shooter mechanisms
     * @param dt elapsed time since last call
     * @param volts current voltage
     * @param runPaddle if true, paddle runs
     */
    public void updateShooter(double dt, double volts, boolean runPaddle) {
        // Move elevator down after it moves slightly up on deploy
        if (timeToElevatorDown < 0) {
            elevator.toggleBottomPos();
        }
        else {
            timeToElevatorDown -= dt;
        }

        // Update hood and elevator
        hood.updateHood();
        elevator.updateElevator(dt, volts);

        // If intial delay is over, run the paddle
        if (runPaddle) {
            paddle.updatePaddle(dt, volts, !elevator.hitBottom());
        }

        // When elevator is movinng down, move paddle to normal resting position
        if (elevator.movingDown()) {
            paddle.toggleNormalRest();
        }
    }

    /**
     * Motion profile function
     * @param maxAccel maximum acceleration
     * @param maxVel maximum velocity
     * @param distance distance to move
     * @param elapsedTime total time elapsed since motion profile started
     * @return array of the calculated distance, velocity, and acceleration
     */
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
