package org.firstinspires.ftc.teamcode;

//import com.acmerobotics.dashboard.telemetry.TelemetryPacket;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.teamcode.Shooter.Launcher;
//import com.acmerobotics.dashboard.telemetry.TelemetryPacket;

@Config
public class Elevator {
    DcMotorEx elevatorMotor;
    TouchSensor elevatorSwitch;
    final double MAX_POWER = 0.5;
    static boolean SLEW_PID = true;
    static double SLEW_RATE = 0.15;
    double MAX_DIST = 13; // inches
    double DIST_TOLERANCE = 1;
    final double NUM_TEETH = 10;
    final double TICKS_PER_REV = 357.7;
    final double GEAR_DIAMETER = 1.158;
    enum STATE {INTAKE, SHOOT, MANUAL, STOP, HOLD_POS};
    STATE elevatorState = STATE.HOLD_POS;
    double integral = 0.0;
    double lastError;
    double holdPos = 0.0;
    double lastPIDOutput = 0.0;
    @Config
    public static class ELEVATOR_PID_COEFFS {
        public static double kP = 0.3;
        public static double kI = 0;
        public static double kD = 0;
        public static double kG = 0.06;
    }

    private double ticksToDist(double ticks) {
        double distPerRev = NUM_TEETH * 8; // 8 mm pitch (dist between chains)
        double distPerRevInch = distPerRev / 25.4;
        return ticks * (distPerRevInch / TICKS_PER_REV);
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

    public void toggleShootPos() {
        elevatorState = STATE.SHOOT;
    }

    public void toggleIntakePos() {
        elevatorState = STATE.INTAKE;
    }

    public void runElevator(double elapsedTime) {
        double currentPos = ticksToDist(elevatorMotor.getCurrentPosition());
        double pidOutput = 0.0;
        double error = 0.0;

        if (hitBottom()) {
            elevatorMotor.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
            elevatorMotor.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        }
        if ((ticksToDist(elevatorMotor.getCurrentPosition()) >= MAX_DIST) && (elevatorState == STATE.SHOOT)) {
            setHoldState();
        }

        if (elevatorState == STATE.STOP) {
            elevatorMotor.setPower(0);
        }
        else if (elevatorState != STATE.MANUAL) {
//            if (hitBottom() && elevatorState == STATE.INTAKE) {
//                elevatorState = STATE.HOLD_POS;
//                holdPos = elevatorMotor.getCurrentPosition();
//                elevatorMotor.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
//                elevatorMotor.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
//            }

            if (elevatorState == STATE.INTAKE) {
                error = 0 - currentPos;
            } else if (elevatorState == STATE.SHOOT) {
                error = MAX_DIST - currentPos;
            } else {
                error = holdPos - currentPos;
            }

            integral += error * elapsedTime;
            double derivative = (error - lastError) / elapsedTime;
            lastError = error;

            pidOutput = ELEVATOR_PID_COEFFS.kP * error +
                    ELEVATOR_PID_COEFFS.kI * integral +
                    ELEVATOR_PID_COEFFS.kD * derivative;

            pidOutput += ELEVATOR_PID_COEFFS.kG;

            if (SLEW_PID) {
                pidOutput = Helpers.slew(SLEW_RATE, pidOutput, lastPIDOutput);
            }
            lastPIDOutput = pidOutput;

            pidOutput = Math.max(-1,
                    Math.min(1, pidOutput));

            elevatorMotor.setPower(pidOutput);

            if (Math.abs(error) < DIST_TOLERANCE && elevatorState == STATE.SHOOT) {
                setHoldState();
            }
            if (Math.abs(error) < DIST_TOLERANCE && elevatorState == STATE.INTAKE) {
                elevatorState = STATE.STOP; // don't hold position, just stop motor
            }
        }

        TelemetryPacket packet = new TelemetryPacket();

        packet.put("Switch pressed", hitBottom());
        packet.put("Elevator State", elevatorState.toString());
        packet.put("Current position", currentPos);
        packet.put("Error", error);
        packet.put("PID Output", pidOutput);

        packet.put("Slew PID On", SLEW_PID);
        packet.put("Slew rate", SLEW_RATE);

        packet.put("kP", ELEVATOR_PID_COEFFS.kP);
        packet.put("kI", ELEVATOR_PID_COEFFS.kI);
        packet.put("kD", ELEVATOR_PID_COEFFS.kD);
        packet.put("kF", ELEVATOR_PID_COEFFS.kG);

//        FtcDashboard.getInstance().sendTelemetryPacket(packet);
    }

    public void goToPos() {
        // go to pos
    }

    public void runForward(double value) {
        elevatorState = STATE.MANUAL;
        elevatorMotor.setPower(Math.clamp(value, 0, MAX_POWER));

        if (ticksToDist(elevatorMotor.getCurrentPosition()) >= MAX_DIST) {
            setHoldState();
        }
    }

    public void runBackward(double value) {
        elevatorState = STATE.MANUAL;
        elevatorMotor.setPower(-Math.clamp(value, 0, MAX_POWER));
    }

    public void rest() {
        if (elevatorState == STATE.MANUAL) {
            setHoldState();
        }
    }

}
