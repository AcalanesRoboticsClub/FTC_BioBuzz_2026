package org.firstinspires.ftc.teamcode.Shooter;

//import com.acmerobotics.dashboard.telemetry.TelemetryPacket;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Helpers;
//import com.acmerobotics.dashboard.telemetry.TelemetryPacket;

@Config
public class Elevator {
    DcMotorEx elevatorMotor;
    TouchSensor elevatorSwitch;
    public static boolean elevatorisUp = false;
    final double MAX_POWER = 0.5;
    public static double FLOWER_HEIGHT = 9.1;
    static boolean SLEW_PID = true;
    static double SLEW_RATE = 0.1;
    public static double MAX_HEIGHT = 9.2; // inches
    double DIST_TOLERANCE = 0.7;
    public static double VEL_TOLERANCE = 0.01;
    final double NUM_TEETH = 10;
    final double TICKS_PER_REV = 537.7; // 1607 * 10 * 8 / (25.4 * 9.875) * 0.9;
    // 538.8
    final double GEAR_DIAMETER = 1.158;
    enum STATE {
        BOTTOM, // goes to bottom
        TOP, // goes to max height
        FLOWER, // goes to flower height
        GO_TO_POS, // goes to an inputted height
        MANUAL, // manual movement using joystick
        STOP, // stops motor
        HOLD_POS, // holds elevator at a constant height
        CREEP}; // creeps downward slowly
    STATE elevatorState = STATE.STOP;
    STATE last_state = STATE.STOP;
    double target_height = 0;

    // PID values
    double integral = 0.0;
    double lastError;
    double lastPower = 0.0;
    double lastPIDOutput = 0.0;
    double holdPos = 0.0;

    // Profile values
    double profileStartPos = 0;
    double profileTarget = 0;
    double profileDir = 1;
    double refPos = 0;
    double refVel = 0;
    double profileDist = 0;
    public static double ELEVATOR_VEL_FILTER_ALPHA = 0.3;
    double filteredVel = 0;
    ElapsedTime elevator_motion_timer = new ElapsedTime();
    public static double elevatorPidOutput = 0;
    double lastPosition = 0;

    // TUNING COEFFICIENTS
    public static double CREEP_POW = -0.01;
    public static double CREEP_HEIGHT = 3.4;
    public static double MAX_VEL = 20;
    public static double MAX_ACCEL = 18;
    @Config
    public static class ELEVATOR_PID_COEFFS {
        public static double kP = 0.023;
        public static double kP_DOWN = 0.01;
        public static double kD_DOWN = 0.0;
        public static double kI = 0;
        public static double kD = 0.003;
        public static double kV = 0.052;
        public static double kA = 0.005;
        public static double kG = 0.1205; // .1628
        public static double kS = 0.2065; // 0.1667
        public static double KP_POS = 0.12; // small correction kP for pose error
        public static double kV_DOWN = 0;
    }

    // Rubber bands
    public static double BAND_REST_LEN = 4.375;   // in, unstretched
    public static double BAND_PRE_LEN = 4.375;   // in, length at elevator bottom
    public static double BAND_RATIO = 1.0;   // band stretch per inch of lift (BAND_RATIO = (L_top − L_bottom) / getPos())
    public static double K_BAND = 0.0293;
    public static double DIST_SCALE = 1.1318;
    public static double DIST_QUAD = -0.0117;

    /*
     * Functions for rubber band calculations
     */
    private double bandShape(double lambda) {
        if (lambda <= 1.0) return 0.0;               // slack
        return lambda - 1.0 / (lambda * lambda);
    }

    private double bandFeedforward(double pos) {
        double lam0 = BAND_PRE_LEN / BAND_REST_LEN;
        double lam  = (BAND_PRE_LEN + BAND_RATIO * pos) / BAND_REST_LEN;
        return K_BAND * (bandShape(lam) - bandShape(lam0));
    }

    /**
     * Gets raw height without accounting for rubber bands
     * @param ticks number of ticks motor has moved
     * @return raw height in inches
     */
    private double rawDist(double ticks) {
        double distPerRevInch = (NUM_TEETH * 8) / 25.4;
        return ticks * (distPerRevInch / TICKS_PER_REV);
    }

    /**
     * Gets height, accounting for rubber bands
     * @param ticks number of ticks motor has moved
     * @return height in inches
     */
    private double ticksToDistInches(double ticks) {
        double raw = rawDist(ticks);
        return raw * (DIST_SCALE + DIST_QUAD * raw);
    }

    /**
     * Gets velocity of elevator, accounting for rubber bands
     * @param ticksPerSec velocity
     * @param ticks number of ticks motor has moved
     * @return velocity of elevator in inch/sec
     */
    private double ticksToVel(double ticksPerSec, double ticks) {
        // derivative of the position correction
        return rawDist(ticksPerSec) * (DIST_SCALE + 2 * DIST_QUAD * rawDist(ticks));
    }

    public Elevator(HardwareMap hardwareMap) {
        elevatorSwitch = hardwareMap.get(TouchSensor.class, "elevatorLimit");
        elevatorMotor = hardwareMap.get(DcMotorEx.class, "elevatorMotor");
        elevatorMotor.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        elevatorMotor.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
//        elevatorMotor.setDirection(DcMotor.Direction.REVERSE);
    }

    /**
     * Sets hold state, where elevator holds current position
     */
    private void setHoldState() {
        elevatorState = STATE.HOLD_POS;
        holdPos = ticksToDistInches(elevatorMotor.getCurrentPosition());
    }

    /**
     * @return true if bottom switch is pressed
     */
    public boolean hitBottom() {
        return elevatorSwitch.isPressed();
    }

    /**
     * @return true if elevator is up
     */
    public static boolean elevatorIsUp() {
        return elevatorisUp;
    }

    /**
     * Checks if elevator is moving down
     * @return true if elevator velocity is negative
     */
    public boolean movingDown() {
        return (elevatorMotor.getPower() < 0);
    }

    /*
     * Toggles different states
     */
    public void toggleTopPos() {
        elevatorState = STATE.TOP;
    }
    public void toggleBottomPos() {
        elevatorState = STATE.BOTTOM;
    }
    public void toggleFlowerPos() {
        elevatorState = STATE.FLOWER;
    }
    public void goToPos(double h) {
        target_height = h;
    }

    /**
     * Manual state upwards motion, clamps power using slew function
     * @param value inputted power value
     */
    public void runForward(double value) {
        elevatorState = STATE.MANUAL;
        double power = Helpers.slew(0.4, Math.clamp(value, 0, MAX_POWER), lastPower);
        elevatorMotor.setPower(Math.clamp(power, 0, MAX_POWER));
        lastPower = power;

//        if (ticksToDistInches(elevatorMotor.getCurrentPosition()) >= MAX_HEIGHT) {
//            setHoldState();
//        }
    }

    /**
     * Manual state downwards motion, clamps power using slew function
     * @param value inputted power value
     */
    public void runBackward(double value) {
        if (elevatorState != STATE.CREEP) {
            elevatorState = STATE.MANUAL;
            double power = Helpers.slew(0.4, -Math.clamp(value, 0, MAX_POWER), lastPower);
            elevatorMotor.setPower(-Math.clamp(power, 0, MAX_POWER));
            lastPower = power;
        }

//        if (ticksToDistInches(elevatorMotor.getCurrentPosition()) <= DIST_TOLERANCE) {
//            elevatorState = STATE.STOP;
//        }
    }

    /**
     * changes from manual to hold states
     * redundant? might remove
     */
    public void rest() {
        if (elevatorState == STATE.MANUAL) {
            setHoldState();
        }
    }

//    public void runElevator2(double elapsedTime, double volts) {
//        elevatorMotor.setPower(elevatorPidOutput);
//
//        TelemetryPacket packet = new TelemetryPacket();
//
//        packet.put("Switch pressed", hitBottom());
//        packet.put("Elevator ticks", elevatorMotor.getCurrentPosition());
//
//        FtcDashboard.getInstance().sendTelemetryPacket(packet);
//    }

    /**
     * Update function - uses states to move to different positions using motion profiles
     * @param elapsedTime time since last function call
     * @param volts battery voltage
     */
    public void updateElevator(double elapsedTime, double volts) {
        // Current position and velocty with a low-pass filter
        double currentPos = ticksToDistInches(elevatorMotor.getCurrentPosition());
        double rawVel = ticksToVel(elevatorMotor.getVelocity(), elevatorMotor.getCurrentPosition());
        filteredVel += ELEVATOR_VEL_FILTER_ALPHA * (rawVel - filteredVel);
        double currentVel = filteredVel;

        double pidOutput = 0.0;
        double error = 0.0;

        // When elevator hits the bottom switch encoder is reset to 0
        if (hitBottom()) {
            elevatorMotor.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
            elevatorMotor.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        }

        // If elevator goes above max height during top PID set to hold state (manual motion overrides this)
        if ((ticksToDistInches(elevatorMotor.getCurrentPosition()) >= MAX_HEIGHT) && (elevatorState == STATE.TOP)) {
            setHoldState();
        }

        // if elevator position is higher than 0.5 inches, elevator is above the ground
        if ((ticksToDistInches(elevatorMotor.getCurrentPosition())) >= 0.5) {
            elevatorisUp = true;
        } else {
            elevatorisUp = false;
        }

        // If change state, start motion profile
        if (elevatorState != last_state) {
            // Sets target height
            if (elevatorState == STATE.GO_TO_POS) {
                profileTarget = target_height;
            } else if (elevatorState == STATE.BOTTOM) {
                profileTarget = CREEP_HEIGHT;
            } else if (elevatorState == STATE.FLOWER) {
                profileTarget = FLOWER_HEIGHT;
            } else if (elevatorState == STATE.TOP) {
                profileTarget = MAX_HEIGHT;
            }

            // starts profile
            profileStartPos = currentPos;
            profileDist = Math.abs(profileTarget - profileStartPos);
            profileDir = Math.signum(profileTarget - profileStartPos);
            elevator_motion_timer.reset();
            integral = 0;
            lastError = 0;
            last_state = elevatorState;
        }

        if (elevatorState == STATE.STOP) { // cuts power
            elevatorMotor.setPower(0);
        } else if (elevatorState == STATE.CREEP) { // creeps downward
            pidOutput = CREEP_POW;
        } else if (elevatorState != STATE.MANUAL) {
            // For hold position use a basic PID
            if (elevatorState == STATE.HOLD_POS) {
                error = holdPos - currentPos;

                integral += error * elapsedTime;
                double derivative = (error - lastError) / elapsedTime;
                lastError = error;

                pidOutput = ELEVATOR_PID_COEFFS.kP * error +
                        ELEVATOR_PID_COEFFS.kI * integral +
                        ELEVATOR_PID_COEFFS.kD * derivative;

                // gravity and rubber bands
                pidOutput += ELEVATOR_PID_COEFFS.kG + bandFeedforward(holdPos);
            }
            // For moving to flower or top position, use a motion profile
            else if (elevatorState == STATE.FLOWER || elevatorState == STATE.TOP) {
                //            if (elevatorState == STATE.BOTTOM) {
                //                error = 0 - currentPos;
                //            } else if (elevatorState == STATE.TOP) {
                //                error = MAX_HEIGHT - currentPos;
                //            } else if (elevatorState == STATE.FLOWER) {
                //                error = FLOWER_HEIGHT - currentPos;
                //            } else {
                //                error = holdPos - currentPos;
                //            }

                double[] profile = Shooter.motion_profile(MAX_ACCEL, MAX_VEL, profileDist, elevator_motion_timer.seconds());

                // predicted values for position, velocity, and acceleration
                refPos = profileStartPos + profileDir * profile[0];
                refVel = profileDir * profile[1];
                double refAcc = profileDir * profile[2];

                // error
                double velError = refVel - currentVel;
                double posError = refPos - currentPos;

                error = profileTarget - currentPos;

                // velocity PID integral and derivative
                integral += velError * elapsedTime;
                double derivative = (velError - lastError) / elapsedTime;
                lastError = velError;

                // TODO: test for when moving down slightly from top position to flower
                // might delete this part now that there is a separate PID for bottom state
                double kV_val = ELEVATOR_PID_COEFFS.kV;
                double kS_val = ELEVATOR_PID_COEFFS.kS;
                double kP_val = ELEVATOR_PID_COEFFS.kP;
                if (refVel < 0) {
                    kS_val = 0.02 * kS_val;
                    kP_val = ELEVATOR_PID_COEFFS.kP_DOWN;
                }

                pidOutput = kS_val * Math.signum(refVel) +
                        kV_val * refVel +
                        ELEVATOR_PID_COEFFS.kA * refAcc +
                        kP_val * velError +
                        ELEVATOR_PID_COEFFS.kI * integral +
                        ELEVATOR_PID_COEFFS.kD * derivative +
                        ELEVATOR_PID_COEFFS.KP_POS * posError;

//                if (Math.abs(velError) < VEL_TOLERANCE && elevatorState == STATE.TOP) {
//                    setHoldState();
//                }
//                if (Math.abs(velError) < VEL_TOLERANCE && elevatorState == STATE.BOTTOM) {
//                    elevatorState = STATE.STOP; // don't hold position, just stop motor
//                }

                double ffPos = (elevatorState == STATE.HOLD_POS) ? holdPos : refPos;
                pidOutput += ELEVATOR_PID_COEFFS.kG + bandFeedforward(ffPos);
            }
            // For moving to bottom position, move down at set speed until hit position to creep downward
            else if (elevatorState == STATE.BOTTOM) {
                // Use PD with kS
                pidOutput = -0.05 * ELEVATOR_PID_COEFFS.kS +
                        ELEVATOR_PID_COEFFS.kP_DOWN * (profileTarget - currentPos) +
                        ELEVATOR_PID_COEFFS.kD_DOWN * (currentPos - lastPosition) / elapsedTime;
            }

            lastPosition = currentPos;

            // Account for batter voltage and clamp output
            pidOutput *= 12.0 / volts;
            pidOutput = Math.max(-1, Math.min(1, pidOutput));

            lastPIDOutput = pidOutput;

            elevatorMotor.setPower(pidOutput);
        }

        // For manual movement, change to creep state regardless of velocity (TODO: fix because currently doesn't work)
        if (ticksToDistInches(elevatorMotor.getCurrentPosition()) < CREEP_HEIGHT && currentVel < 0 && elevatorState == STATE.MANUAL) {
            elevatorState = STATE.CREEP;
        }

        // For manual, hold_pos, or bottom states switch to creep state when hits creep height and velocity is not too fast
        if (ticksToDistInches(elevatorMotor.getCurrentPosition() - CREEP_HEIGHT) < DIST_TOLERANCE && (currentVel < 0.0 && currentVel > -2.0) && (elevatorState == STATE.MANUAL || elevatorState == STATE.BOTTOM || elevatorState == STATE.HOLD_POS)) {
            elevatorState = STATE.CREEP; // creep down
        }

        // once almost to the bottom and velocity is low stop motor
        if (ticksToDistInches(elevatorMotor.getCurrentPosition()) < DIST_TOLERANCE && Math.abs(currentVel) < 2.0 && (elevatorState == STATE.MANUAL || elevatorState == STATE.CREEP || elevatorState == STATE.HOLD_POS)) {
            elevatorState = STATE.STOP; // don't hold position, just stop motor
        }

        TelemetryPacket packet = new TelemetryPacket();

        packet.put("Switch pressed", hitBottom());
        packet.put("Elevator State", elevatorState.toString());
//        packet.put("Elevator ticks", elevatorMotor.getCurrentPosition());
        packet.put("Elevator Current position", currentPos);
//        packet.put("Elevator Reference position", refPos);
//        packet.put("Elevator Reference velocity", refVel);
//        packet.put("Elevator current velocity", currentVel);
//        packet.put("Error", error);
//        packet.put("PID Output", pidOutput);
//        packet.put("Target position", profileTarget);

//        packet.put("Slew PID On", SLEW_PID);
//        packet.put("Slew rate", SLEW_RATE);
//
//        packet.put("kP", ELEVATOR_PID_COEFFS.kP);
//        packet.put("kI", ELEVATOR_PID_COEFFS.kI);
//        packet.put("kD", ELEVATOR_PID_COEFFS.kD);
//        packet.put("kF", ELEVATOR_PID_COEFFS.kG);

        FtcDashboard.getInstance().sendTelemetryPacket(packet);
    }

}
