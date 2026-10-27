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
    double MAX_DIST = 12; // inches
    double DIST_TOLERANCE = 1;
    final double NUM_TEETH = 10;
    final double TICKS_PER_REV = 357.7;
    final double GEAR_DIAMETER = 1.158;
    enum STATE {INTAKE, SHOOT, MANUAL, STOP, HOLD_POS};
    STATE elevatorState = STATE.INTAKE;
    double integral = 0.0;
    double lastError;
    @Config
    public static class ELEVATOR_PID_COEFFS {
        public static double kP = 1;
        public static double kI = 0;
        public static double kD = 0;
        public static double kF = 0;
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

        if (elevatorState == STATE.STOP) {
            elevatorMotor.setPower(0);
        }
        else if (elevatorState == STATE.HOLD_POS) {
            elevatorMotor.setPower(ELEVATOR_PID_COEFFS.kF);
        }
        else if (elevatorState != STATE.MANUAL) {
            if (hitBottom() && elevatorState == STATE.INTAKE) {
                elevatorState = STATE.STOP;
                elevatorMotor.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
                elevatorMotor.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
            }

            if (elevatorState == STATE.INTAKE) {
                error = 0 - currentPos;
            } else if (elevatorState == STATE.SHOOT) {
                error = MAX_DIST - currentPos;
            } else {
                error = 0;
            }

            integral += error * elapsedTime;
            double derivative = (error - lastError) / elapsedTime;
            lastError = error;

            pidOutput = ELEVATOR_PID_COEFFS.kP * error +
                    ELEVATOR_PID_COEFFS.kI * integral +
                    ELEVATOR_PID_COEFFS.kD * derivative;

            // Static friction (overcomes initial friction to move)
            double kS = 0.05;

            // Velocity & Acceleration Feedforward terms
            double kV = 0.001;
            double kA = 0.0001;

            // Gravity compensation term
            double kG = 0.1;

            //        double f = (Math.signum(targetVelocity) * kS)
            //                + (kV * targetVelocity)
            //                + (kA * targetAccel)
            //                + kG;
            //
            pidOutput += ELEVATOR_PID_COEFFS.kF;

//            elevatorMotor.setPower(pidOutput);

            if (error < DIST_TOLERANCE && elevatorState == STATE.SHOOT) {
                elevatorState = STATE.HOLD_POS;
            }
        }

        TelemetryPacket packet = new TelemetryPacket();

        packet.put("Switch pressed", hitBottom());
        packet.put("Elevator State", elevatorState.toString());
        packet.put("Current position", currentPos);
        packet.put("Error", error);
        packet.put("PID Output", pidOutput);

        packet.put("kP", ELEVATOR_PID_COEFFS.kP);
        packet.put("kI", ELEVATOR_PID_COEFFS.kI);
        packet.put("kD", ELEVATOR_PID_COEFFS.kD);
        packet.put("kF", ELEVATOR_PID_COEFFS.kF);

        FtcDashboard.getInstance().sendTelemetryPacket(packet);
    }

    public void goToPos() {
        // go to pos
    }

    public void runForward(double value) {
        elevatorState = STATE.MANUAL;
        elevatorMotor.setPower(value);
    }

    public void runBackward(double value) {
        elevatorState = STATE.MANUAL;
        elevatorMotor.setPower(-value);
    }

    public void rest() {
        elevatorState = STATE.STOP;
    }

}
