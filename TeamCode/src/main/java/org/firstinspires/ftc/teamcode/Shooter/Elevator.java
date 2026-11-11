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
    final double FLOWER_HEIGHT = 8.6; // actual is 9.4
    static boolean SLEW_PID = true;
    static double SLEW_RATE = 0.1;
    double MAX_HEIGHT = 8.6; // inches
    double DIST_TOLERANCE = 1;
    public static double VEL_TOLERANCE = 0.01;
    final double NUM_TEETH = 10;
    final double TICKS_PER_REV = 1607 * 9.875 * 8 / (25.4*10) * 8/7; // 357.7
    //
    final double GEAR_DIAMETER = 1.158;
    enum STATE {BOTTOM, TOP, FLOWER, MANUAL, STOP, HOLD_POS};
    STATE elevatorState = STATE.STOP;
    STATE last_state = STATE.STOP;

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
    double profileDist = 0;
    ElapsedTime elevator_motion_timer = new ElapsedTime();

    // TUNING COEFFICIENTS
    public static double MAX_VEL = 5;
    public static double MAX_ACCEL = 3;
    @Config
    public static class ELEVATOR_PID_COEFFS {
        public static double kP = 0.01;
        public static double kI = 0;
        public static double kD = 0;
        public static double kV = 0.3;
        public static double kA = 0;
        public static double kG = 0.23;
        public static double KP_POS = 0.8; // small correction kP for pose error
    }

    private double ticksToDist(double ticks) {
        double distPerRev = NUM_TEETH * 8; // 8 mm pitch (dist between chains)
        double distPerRevInch = distPerRev / 25.4;
        return ticks * (distPerRevInch / TICKS_PER_REV);
    }

    public static boolean elevatorIsUp() {
        return elevatorisUp;
    }

    public Elevator(HardwareMap hardwareMap) {
        elevatorSwitch = hardwareMap.get(TouchSensor.class, "elevatorLimit");
        elevatorMotor = hardwareMap.get(DcMotorEx.class, "elevatorMotor");
        elevatorMotor.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        elevatorMotor.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
//        elevatorMotor.setDirection(DcMotor.Direction.REVERSE);
    }

    private void setHoldState() {
        elevatorState = STATE.HOLD_POS;
        holdPos = ticksToDist(elevatorMotor.getCurrentPosition());
    }

    public boolean hitBottom() {
        return elevatorSwitch.isPressed();
    }

    public void toggleTopPos() {
        elevatorState = STATE.TOP;
    }

    public void toggleBottomPos() {
        elevatorState = STATE.BOTTOM;
    }

    public void toggleFlowerPos() {
        elevatorState = STATE.FLOWER;
    }

    public void runElevator(double elapsedTime) {
        double currentPos = ticksToDist(elevatorMotor.getCurrentPosition());
        double currentVel = ticksToDist(elevatorMotor.getVelocity());
        double pidOutput = 0.0;
        double error = 0.0;

        if (hitBottom()) {
            elevatorMotor.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
            elevatorMotor.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        }
        if ((ticksToDist(elevatorMotor.getCurrentPosition()) >= MAX_HEIGHT) && (elevatorState == STATE.TOP)) {
            setHoldState();
        }

        if ((ticksToDist(elevatorMotor.getCurrentPosition())) >= 0.5) {
            elevatorisUp = true;
        }
        else {
            elevatorisUp = false;
        }

        // If change state, start motion profile
        if (elevatorState != last_state) {
            if (elevatorState == STATE.BOTTOM) {
                profileStartPos = currentPos;
                profileTarget = 0;
                profileDir = -1;
                elevator_motion_timer.reset();
            }
            else if (elevatorState == STATE.FLOWER) {
                profileStartPos = currentPos;
                profileTarget = FLOWER_HEIGHT;
                profileDir = 1;
                elevator_motion_timer.reset();
            }
            else if (elevatorState == STATE.TOP) {
                profileStartPos = currentPos;
                profileTarget = MAX_HEIGHT;
                profileDir = 1;
                elevator_motion_timer.reset();
            }
            profileDist = Math.abs(profileTarget - profileStartPos);
            integral = 0;
            lastError = 0;
            last_state = elevatorState;
        }

        if (elevatorState == STATE.STOP) {
            elevatorMotor.setPower(0);
        }
        else if (elevatorState != STATE.MANUAL) {
            if (elevatorState == STATE.HOLD_POS) {
                error = holdPos - currentPos;

                integral += error * elapsedTime;
                double derivative = (error - lastError) / elapsedTime;
                lastError = error;

                pidOutput = ELEVATOR_PID_COEFFS.kP * error +
                        ELEVATOR_PID_COEFFS.kI * integral +
                        ELEVATOR_PID_COEFFS.kD * derivative;
            }
            else {
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

                // predicted values
                refPos = profileStartPos + profileDir * profile[0];
                double refVel = profileDir * profile[1];
                double refAcc = profileDir * profile[2];

                // error
                double velError = refVel - currentVel;
                double posError = refPos - currentPos;

                error = profileTarget - currentPos;

                // velocity PID
                integral += velError * elapsedTime;
                double derivative = (velError - lastError) / elapsedTime;
                lastError = velError;

                pidOutput = ELEVATOR_PID_COEFFS.kV * refVel +
                        ELEVATOR_PID_COEFFS.kA * refAcc +
                        ELEVATOR_PID_COEFFS.kP * velError +
                        ELEVATOR_PID_COEFFS.kI * integral +
                        ELEVATOR_PID_COEFFS.kD * derivative +
                        ELEVATOR_PID_COEFFS.KP_POS * posError;

//                if (Math.abs(velError) < VEL_TOLERANCE && elevatorState == STATE.TOP) {
//                    setHoldState();
//                }
//                if (Math.abs(velError) < VEL_TOLERANCE && elevatorState == STATE.BOTTOM) {
//                    elevatorState = STATE.STOP; // don't hold position, just stop motor
//                }
            }

            pidOutput += ELEVATOR_PID_COEFFS.kG;

            if (SLEW_PID) {
                pidOutput = Helpers.slew(SLEW_RATE, pidOutput, lastPIDOutput);
            }
            lastPIDOutput = pidOutput;

            pidOutput = Math.max(-1,
                    Math.min(1, pidOutput));

            elevatorMotor.setPower(pidOutput);
        }

        if (ticksToDist(elevatorMotor.getCurrentPosition()) < DIST_TOLERANCE && (elevatorState == STATE.MANUAL || elevatorState == STATE.BOTTOM || elevatorState == STATE.HOLD_POS)) {
            elevatorState = STATE.STOP; // don't hold position, just stop motor
        }

        TelemetryPacket packet = new TelemetryPacket();

        packet.put("Switch pressed", hitBottom());
        packet.put("Elevator State", elevatorState.toString());
        packet.put("Elevator ticks", elevatorMotor.getCurrentPosition());
        packet.put("Elevator Current position", currentPos);
        packet.put("Elevator Reference position", refPos);
//        packet.put("Elevator Reference position", refPos);
        packet.put("Error", error);
        packet.put("PID Output", pidOutput);

//        packet.put("Slew PID On", SLEW_PID);
//        packet.put("Slew rate", SLEW_RATE);

        packet.put("kP", ELEVATOR_PID_COEFFS.kP);
        packet.put("kI", ELEVATOR_PID_COEFFS.kI);
        packet.put("kD", ELEVATOR_PID_COEFFS.kD);
        packet.put("kF", ELEVATOR_PID_COEFFS.kG);

        FtcDashboard.getInstance().sendTelemetryPacket(packet);
    }

    public void goToPos() {
        // go to pos
    }

    public void runForward(double value) {
        elevatorState = STATE.MANUAL;
        double power = Helpers.slew(0.4, Math.clamp(value, 0, MAX_POWER), lastPower);
        elevatorMotor.setPower(Math.clamp(power, 0, MAX_POWER));
        lastPower = power;

//        if (ticksToDist(elevatorMotor.getCurrentPosition()) >= MAX_HEIGHT) {
//            setHoldState();
//        }
    }

    public void runBackward(double value) {
        elevatorState = STATE.MANUAL;
        double power = Helpers.slew(0.4, -Math.clamp(value, 0, MAX_POWER), lastPower);
        elevatorMotor.setPower(-Math.clamp(power, 0, MAX_POWER));
        lastPower = power;

//        if (ticksToDist(elevatorMotor.getCurrentPosition()) <= DIST_TOLERANCE) {
//            elevatorState = STATE.STOP;
//        }
    }

    public void rest() {
        if (elevatorState == STATE.MANUAL) {
            setHoldState();
        }
    }

}
