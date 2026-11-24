package org.firstinspires.ftc.teamcode.Shooter;

//import com.acmerobotics.dashboard.telemetry.TelemetryPacket;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;

//import com.acmerobotics.dashboard.telemetry.TelemetryPacket;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;

@Config
public class Paddle {
    DcMotorEx paddleMotor;

    final double NUDGE_ZERO_AMOUNT = 1;
    double paddleZeroPos = 0.0;

    // Acceleration limit
    static boolean SLEW_PID = false;
    static double SLEW_RATE = 0.15;

    // Angle constants
    final double REST_ANGLE = 270.0; // help guide objects into sleeve
    final double REST_UP_ANGLE = 300.0; // prevent objects from falling out
    final double SHOOT_POS_ANGLE = 60.0;
    final double SHOOT_ANGLE = 360 * 2.75;
    final double FLOWER_ANGLE = 360 * 1.2;

    // Mechanism constants
    final double GEAR_RATIO = 10.0 / 24.0;
    final double TICKS_PER_REV = 224.0; // gobilda - 357.7?

    // Tolerances
    public static double ANGLE_TOLERANCE_TO_SHOOT = 10;
    public static double ANGLE_TOLERANCE_TO_REST = 10;
    double targetAngle = 0;

    // STATES
    public enum LAUNCH_STATE {
        REST, // normal rest position
        REST_ELEVATOR_UP, // rest position when elevator is up
        POSITION, // move backwards a position to give shoot profile more time to get up to speed
        SHOOT, // shoot (either flower or regular shoot)
        SPIT}; // reverse direction to spit out game objects
    public static LAUNCH_STATE launch_state = LAUNCH_STATE.REST;
    public enum LAUNCH_TYPE {FLOWER, SHOOT};
    public static LAUNCH_TYPE launchType = LAUNCH_TYPE.SHOOT;
    boolean shootToggle = false;
    boolean elevatorUpReadyForProfile = true;

    // Profile values
    double lastTargetAngle = 0;
    double profileStartAngle = 0;
    double profileTargetAngle = 0;
    double profileDir = 1;
    double refAngle = 0;
    ElapsedTime paddle_motion_timer = new ElapsedTime();

    // Shooting profile values
    double shootRefVel = 0;
    double filteredVel = 0; // TODO: change to kalman filter to smooth motion
    public static double LAUNCH_VEL_FILTER_ALPHA = 0.7;
    ElapsedTime shootTimer = new ElapsedTime();
    double shootDist = 0;
    public static double shootVelocity = 2000;
    public double profileShootVelocity = 2000; // during shot profile keep target velocity constant

    // PID TUNING COEFFICIENTS
    @Config
    public static class PADDLE_PID_COEFFS {
        public static double kS = 0.0546;
        public static double kV = 0.0002;
        public static double kA = 0.00006;
        public static double kP = 0.0001;
        public static double kI = 0;
        public static double kD = 0.00001;
        public static double kG = 0.04345; // (-0.0979-0.011)/2,    0.0436
        public static double KP_POS = 0.003;
        public static double KP_HOLD_POS = 0.008;
        public static double kLOAD = 0.00;  // extra constant to account for game objects
    }
    @Config
    public static class PADDLE_SHOOT_PID_COEFFS {
        public static double kI = 0;
        public static double kS = 0.05445;
        public static double kV = 0.0002;
        public static double kA = 0.00005; // 0.006
        public static double kP = 0.0004; // 0.002
        public static double kD = 0;
        public static double kG = 0.0436;
        public static double KP_POS = 0;
        public static double KP_HOLD_POS = 0;
    }
    @Config
    public static class PADDLE_FLOWER_PID_COEFFS {
        public static double kI = 0;
        public static double kS = 0.05445;
        public static double kV = 0.0002;
        public static double kA = 0.002;
        public static double kP = 0.0007;
        public static double kD = 0;
        public static double kG = 0.0436;
        public static double KP_POS = 0;
        public static double KP_HOLD_POS = 0;
    }
    double lastError = 0.0;
    double integral = 0.0;
    double velIntegral = 0;
    public static double pidOutput;
    double lastPIDOutput = 0.0;

    // Profile constants
    public static double MAX_VEL = 1200;
    public static double MAX_ACCEL = 800;
    public static double I_MAX = 0.25;

    // Shoot profile constants
    public static double FLOWER_VELOCITY = 100; // deg/sec
    public static double SHOOT_MAX_VELOCITY = 2200; // clamp max velocity
    public static double FLOWER_SHOOT_ACCEL = 800;
    public static double SHOOT_ACCEL_DIST = 230;
    public static double SHOOT_CRUISE_DIST = 200;
    public static double FLOWER_ACCEL_DIST = 25;
    public static double FLOWER_CRUISE_DIST = 270;
    double lastShootError = 0.0;

    /**
     * converts from degrees to ticks
     * @param degrees degrees paddle has rotated
     * @return ticks motor has moved
     */
    private double degreesToTicks(double degrees) {
        return (degrees * (TICKS_PER_REV * GEAR_RATIO / 360.0));
    }

    /**
     * converts ticks to degrees
     * @param ticks number of ticks motor has moved
     * @return degrees rotated
     */
    private double ticksToDegrees(double ticks) {
//        double degrees = (ticks * 360.0 / (TICKS_PER_REV * GEAR_RATIO));
        double degrees = (ticks * 360.0 / (TICKS_PER_REV)) + paddleZeroPos;
//        if (degrees < 0) {
//            degrees += 360.0;
//        }
        return degrees;
    }

    /**
     * Gets velocity of launcher in deg/sec
     * @param ticksPerSec velocity in ticks/sec
     * @return velocity in deg/sec
     */
    private double ticksPerSecToDegPerSec(double ticksPerSec) {
        return ticksPerSec * 360.0 / TICKS_PER_REV;
    }

    public Paddle(HardwareMap hardwareMap) {
        paddleMotor = hardwareMap.get(DcMotorEx.class, "launchMotor");
//        paddleMotor.setDirection(DcMotorEx.Direction.REVERSE);
        paddleMotor.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        paddleMotor.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        paddleZeroPos = 180; // zero position is straight up
    }

    /*
     * Getter functions
     */
    public double getCurrentAngle() { // get angle
        return ticksToDegrees(paddleMotor.getCurrentPosition());
    }
    public double getCurrentTicks() { // get ticks
        return paddleMotor.getCurrentPosition();
    }
    public double getOutput() { // get power output
        return paddleMotor.getPower();
    }

    /*
     * Toggles paddle states
     */
    public void toggleRest() {
        launch_state = LAUNCH_STATE.REST;
    }
    public void toggleShoot() {
        shootToggle = true;
    }
    public void spit() {
        launch_state = LAUNCH_STATE.SPIT;
        shootToggle = false;
    }

    /**
     * Starts profile to move back to normal resting position (270 degrees)
     */
    public void toggleNormalRest() {
        double currentAngle = getCurrentAngle();
        targetAngle = currentAngle + (REST_ANGLE - (currentAngle % 360));
        startProfile(currentAngle);
        launch_state = LAUNCH_STATE.REST;
    }

    /*
     * Toggles paddle velocities
     */
    public void toggleFlowerVelocity() {
        launchType = LAUNCH_TYPE.FLOWER;
        shootRefVel = FLOWER_VELOCITY;
    }
    public void toggleShootVelocity() {
        launchType = LAUNCH_TYPE.SHOOT;
        shootRefVel = Math.min(Math.abs(shootVelocity), SHOOT_MAX_VELOCITY);
    }

    /**
     * Sets shoot velocity
     * @param velocity velocity (calculated in shooter with setShootProfile)
     */
    public void setShootVelocity(double velocity) {
        shootRefVel = Math.min(Math.abs(velocity), SHOOT_MAX_VELOCITY);
    }

    /**
     * At start of match (after a delay), moves from upright position to rest position
     */
    public void deploy() {
        shootRefVel = shootVelocity;
        targetAngle = REST_ANGLE;
        launch_state = LAUNCH_STATE.REST;
        startProfile(getCurrentAngle());
    }

    /**
     * Starts motion profile for position PID
     * @param currentAngle angle paddle is currently at
     */
    public void startProfile(double currentAngle) {
        profileStartAngle = currentAngle;
        profileTargetAngle = targetAngle;
        profileDir = Math.signum(targetAngle - currentAngle); // moving forward or backward?
        paddle_motion_timer.reset();
        integral = 0;
        lastError = 0;
        lastTargetAngle = targetAngle;
    }

    /**
     * Starts profile to shoot using velocity PID
     * @param currentAngle angle paddle is currently at
     */
    public void startShootProfile(double currentAngle) {
        profileStartAngle = currentAngle;
        profileTargetAngle = targetAngle;
        profileDir = Math.signum(targetAngle - currentAngle); // direction of motion
        integral = 0;
        lastShootError = 0;
        lastTargetAngle = targetAngle;
//        shootDist = targetAngle - currentAngle;
        shootDist = 0; // shoot distance travelled
        profileShootVelocity = shootVelocity;
        shootTimer.reset();
    }

    // TODO: FUNCTION TO SEND LAUNCHER TO BOTTOM AT END OF AUTO

//    public void updatePaddle(double elapsedTime, double volts, boolean elevatorIsUp) {
//        paddleMotor.setPower(pidOutput);
//    }

    /**
     * Update function - uses states to move and shoot using motion profiles
     * @param elapsedTime time since last function call
     * @param volts battery voltage
     * @param elevatorIsUp elevator is above the ground
     */
    public void updatePaddle(double elapsedTime, double volts, boolean elevatorIsUp) {
        // Get current angle and velocity (velocity uses low-pass filter to smooth out motion)
        double currentAngle = getCurrentAngle();
        double rawVel = ticksPerSecToDegPerSec(paddleMotor.getVelocity());
        filteredVel += LAUNCH_VEL_FILTER_ALPHA * (rawVel - filteredVel);
        double currentVel = filteredVel;

        // Joystick button pressed to toggle shoot and start profile
        if (shootToggle) {
            // For flower immediately shoot
            if (launchType == LAUNCH_TYPE.FLOWER) {
                targetAngle = currentAngle + (FLOWER_ANGLE);
                launch_state = LAUNCH_STATE.SHOOT;
                startShootProfile(currentAngle);
            }
            // For normal shoot, first go to POSITION state and spin backwards before shooting
            else {
                targetAngle = currentAngle - (((currentAngle % 360.0) - SHOOT_POS_ANGLE));
                launch_state = LAUNCH_STATE.POSITION;
                startProfile(currentAngle);
            }
            shootToggle = false;
        }

        // When elevator is moved up, move paddle down to keep game objects in
        if (elevatorIsUp && elevatorUpReadyForProfile && launch_state == LAUNCH_STATE.REST) {
            launch_state = LAUNCH_STATE.REST_ELEVATOR_UP;
            targetAngle = currentAngle + (REST_UP_ANGLE - (currentAngle % 360));
            startProfile(currentAngle);
            elevatorUpReadyForProfile = false;
        }
        // when elevator is moved down, move back to normal rest state
        else if (!elevatorIsUp) {
            if (launch_state == LAUNCH_STATE.REST_ELEVATOR_UP) {
                toggleNormalRest();
            }
            elevatorUpReadyForProfile = true;
        }

        // Calculate error
        double error = targetAngle - currentAngle;
        pidOutput = 0.0;
        double refVel = 0.0;

        // SPIT state: move in reverse direction
        if (launch_state == LAUNCH_STATE.SPIT) {
            pidOutput = -0.5;
            launch_state = LAUNCH_STATE.REST; // set to rest so that only continues spit when button is held down
        }
        // SHOOT state: start motion profile
        else if (launch_state == LAUNCH_STATE.SHOOT) {
//            double v = shootRefVel;
            double[] sp;

            double accel;
            double refAcc;
            boolean profile_done = false;
            double pid_load = 0; // account for load of game objects
            double pid_I = 0;
            double velError = 0;
            // For flower currently does a basic motion profile with flower shoot acceleration
            // might swap to shoot profile version with accel dist later
            if (launchType == LAUNCH_TYPE.FLOWER) {
                shootDist = currentAngle - profileStartAngle;
                sp = Shooter.distance_profile(FLOWER_ACCEL_DIST, FLOWER_CRUISE_DIST, FLOWER_VELOCITY, 0.5*FLOWER_VELOCITY, shootDist);

                // predicted angle, velocity, and acceleration
                refAngle = currentAngle; // not part of shoot profile
                refVel = sp[0];
                refAcc = sp[1];
                profile_done = (sp[2] == 1);

                pid_load = PADDLE_PID_COEFFS.kLOAD;
                velError = refVel - currentVel;

                // integral term (for flower)
                velIntegral += velError * elapsedTime;
                double lim = I_MAX / Math.max(PADDLE_FLOWER_PID_COEFFS.kI, 1e-9);
                velIntegral = Math.max(-lim, Math.min(lim, velIntegral));
                pid_I = Math.max(-0.3, Math.min(0.3, velIntegral*PADDLE_FLOWER_PID_COEFFS.kI));
            }
            // For shoot uses a set acceleration distance to calculate the acceleration
            else {
                shootDist = currentAngle - profileStartAngle; // distance travelled since start
                sp = Shooter.distance_profile(SHOOT_ACCEL_DIST, SHOOT_CRUISE_DIST, shootRefVel, 50, shootDist); // set initial velocity to 50 deg/s

                // predicted angle, velocity, and acceleration
                refAngle = currentAngle; // not part of shoot profile
                refVel = sp[0];
                refAcc = sp[1];
                profile_done = (sp[2] == 1);
                velError = refVel - currentVel;

                if (shootDist > SHOOT_ACCEL_DIST && shootDist < SHOOT_ACCEL_DIST + SHOOT_CRUISE_DIST) {
                    pid_load = PADDLE_PID_COEFFS.kLOAD;
                }
            }

            double derivative = (velError - lastShootError) / elapsedTime;
            lastShootError = velError;

            if (!profile_done) {
                if (launchType == LAUNCH_TYPE.FLOWER) {
                    pidOutput = PADDLE_FLOWER_PID_COEFFS.kS * Math.signum(refVel) +
                            PADDLE_FLOWER_PID_COEFFS.kV * refVel +
                            PADDLE_FLOWER_PID_COEFFS.kA * refAcc +
                            PADDLE_FLOWER_PID_COEFFS.kP * velError +
                            PADDLE_FLOWER_PID_COEFFS.kD * derivative +
                            PADDLE_FLOWER_PID_COEFFS.KP_POS * (refAngle - currentAngle) +
                            pid_load +
                            pid_I;
                } else {
                    pidOutput = PADDLE_SHOOT_PID_COEFFS.kS * Math.signum(refVel) +
                            PADDLE_SHOOT_PID_COEFFS.kV * refVel +
                            PADDLE_SHOOT_PID_COEFFS.kA * refAcc +
                            PADDLE_SHOOT_PID_COEFFS.kP * velError +
                            PADDLE_SHOOT_PID_COEFFS.kD * derivative +
                            PADDLE_SHOOT_PID_COEFFS.KP_POS * (refAngle - currentAngle) +
                            pid_load;
                }
            }
            else {
                pidOutput = 0;

                // once paddle has slowed down, change to rest profile
                if (currentVel < 100) {
                    launch_state = LAUNCH_STATE.REST;
                    targetAngle = currentAngle + (REST_ANGLE - (currentAngle % 360));
                    startProfile(currentAngle);
                    elevatorUpReadyForProfile = false;
                }
            }

        }
        // POSITION state: moves backward to a position before shooting
        // REST state: moves to a set position to hold
        else {
            // Motion profile using the distance to travel and MAX_ACCEL and MAX_VEL
            double dist = Math.abs(profileTargetAngle - profileStartAngle);
            double[] profile = Shooter.motion_profile(MAX_ACCEL, MAX_VEL, dist, paddle_motion_timer.seconds());

            // predicted values
            refAngle = profileStartAngle + profileDir * profile[0];
            refVel = profileDir * profile[1];
            double refAcc = profileDir * profile[2];

            // errors
            double velError = refVel - currentVel;
            double posError = refAngle - currentAngle;

            final double I_ZONE = 40;        // only integrate within 40 deg of target
            final double I_MAX_POWER = 0.2;  // integral only contributes max of 0.2 power

            // Only uses integral if the error is large
            if (Math.abs(error) < I_ZONE) {
                integral += error * elapsedTime;
                if (PADDLE_PID_COEFFS.kI != 0) {
                    double lim = I_MAX_POWER / PADDLE_PID_COEFFS.kI;
                    integral = Math.max(-lim, Math.min(lim, integral));
                }
            } else {
                integral = 0;
            }

            // derivative
            double derivative = (velError - lastError) / elapsedTime;
            lastError = velError;

            // for rest state once the position gets close lower KP_POS to reduce oscillation
            double pos_const = PADDLE_PID_COEFFS.KP_POS;
            if (launch_state == LAUNCH_STATE.REST && posError < ANGLE_TOLERANCE_TO_REST) {
                pos_const = PADDLE_PID_COEFFS.KP_HOLD_POS;
            }

            pidOutput = PADDLE_PID_COEFFS.kS * Math.signum(refVel) +
                    PADDLE_PID_COEFFS.kV * refVel +
                    PADDLE_PID_COEFFS.kA * refAcc +
                    PADDLE_PID_COEFFS.kP * velError +
                    PADDLE_PID_COEFFS.kI * integral +
                    PADDLE_PID_COEFFS.kD * derivative +
                    pos_const * posError;
        }

        // Add gravity coefficient
        pidOutput += PADDLE_PID_COEFFS.kG * Math.sin(Math.toRadians(currentAngle % 360));

        // Accounts for battery voltage and clamps it
        pidOutput *= 12.0 / volts;
        pidOutput = Math.max(-1, Math.min(1, pidOutput));

//        if (SLEW_PID) {
//            pidOutput = Helpers.slew(SLEW_RATE, pidOutput, lastPIDOutput);
//        }
        lastPIDOutput = pidOutput;

        paddleMotor.setPower(pidOutput);

        // Once position profile gets close to correct angle, change to SHOOT state and start profile
        if (Math.abs(error) < ANGLE_TOLERANCE_TO_SHOOT && launch_state == LAUNCH_STATE.POSITION) {
            targetAngle = targetAngle + (SHOOT_ACCEL_DIST*2 + SHOOT_CRUISE_DIST);
            launch_state = LAUNCH_STATE.SHOOT;
            startShootProfile(currentAngle);
        }
        // Once shoot profile gets starts slowing down, change to REST state and start profile
        // TODO: try different values for refVel < 25
//        else if (shootTimer.seconds() > 2 && shootDist > SHOOT_ACCEL_DIST + SHOOT_CRUISE_DIST) && currentVel < 500 && launch_state == LAUNCH_STATE.SHOOT) {
////        else if (shootTimer.seconds() > 2 && refVel < 15 && launch_state == LAUNCH_STATE.SHOOT) {
//            launch_state = LAUNCH_STATE.REST;
//            targetAngle = currentAngle + (REST_ANGLE - (currentAngle % 360));
//            startProfile(currentAngle);
//            elevatorUpReadyForProfile = false;
//        }

        TelemetryPacket packet = new TelemetryPacket();
//        packet.put("Current Angle (360)", currentAngle % 360);
//        packet.put("Current Angle", currentAngle);
        packet.put("Target Angle", targetAngle);
        packet.put("Ref Angle", refAngle);
        packet.put("Ref Vel", refVel);
        packet.put("current vel", currentVel);
//        packet.put("Shoot Ref Dist", shootDist);
//        packet.put("Error", error);
        packet.put("PIDOutput", pidOutput);
        packet.put("shoot velocity", shootVelocity);
        packet.put("paddle state", launch_state);
//        packet.put("paddle velocity", ticksToDegrees(paddleMotor.getVelocity()));
        FtcDashboard.getInstance().sendTelemetryPacket(packet);
    }

    /*
     * shift zero position manually to account for encoder getting off as match progresses
     */
    public void shiftZeroPositionForward() {
        paddleZeroPos += NUDGE_ZERO_AMOUNT;
    }
    public void shiftZeroPositionBackward() {
        paddleZeroPos -= NUDGE_ZERO_AMOUNT;
    }

    // functions for testing, just move paddle forward or stop it
    public void paddleFoward() {
        paddleMotor.setPower(1);
    }

    public void paddleForwardSlow() {
        paddleMotor.setPower(0.25);
    }

    public void rest() {
        paddleMotor.setPower(0);
    }

    /**
     * Forces a shoot by skipping to SHOOT state if the position PID doesn't fully complete
     * Changes to SHOOT state and starts profile
     */
    public void shootState() {
        double currentAngle = getCurrentAngle();
        launch_state = LAUNCH_STATE.SHOOT;
        targetAngle = currentAngle + ((REST_ANGLE - (currentAngle % 360)) % 360.0 + 360);
        shootToggle = false;
        startShootProfile(currentAngle);
    }

}
