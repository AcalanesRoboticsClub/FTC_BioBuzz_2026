package org.firstinspires.ftc.teamcode.Shooter;

//import com.acmerobotics.dashboard.telemetry.TelemetryPacket;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.PIDCoefficients;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
//import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import org.firstinspires.ftc.teamcode.Helpers;
import org.firstinspires.ftc.teamcode.Robot;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;

import com.bylazar.ftcontrol.panels.Panels;
import com.bylazar.ftcontrol.panels.integration.TelemetryManager;

@Config
public class Launcher {
    DcMotorEx launchMotor;

    final double NUDGE_ZERO_AMOUNT = 1;
    double launcherZeroPos = 0.0;

    // Acceleration limit
    static boolean SLEW_PID = true;
    static double SLEW_RATE = 0.15;

    // Angle constants
    final double REST_ANGLE = 260.0;
    final double SHOOT_POS_ANGLE = 50.0;
    final double SHOOT_ANGLE = 360 * 2.5;
    final double FLOWER_ANGLE = 360 * 1.2;

    // Mechanism constants
    final double GEAR_RATIO = 10.0 / 24.0;
    final double TICKS_PER_REV = 220.0; // gobilda - 357.7?

    // Tolerances
    double ANGLE_TOLERANCE_TO_SHOOT = 20;
    double ANGLE_TOLERANCE_TO_REST = 30;
    double targetAngle = 0;

    // Shoot power constants
    final double FLOWER_POWER = 0.8;
    final double SHOOT_POWER = 1;
    double shoot_power = 1;

    // STATES
    public enum LAUNCH_STATE {REST, POSITION, SHOOT, SPIT};
    public static LAUNCH_STATE launch_state = LAUNCH_STATE.REST;
    boolean shootToggle = false;

    // Profile values
    double lastTargetAngle = 0;
    double profileStartAngle = 0;
    double profileTargetAngle = 0;
    double profileDir = 1;
    ElapsedTime launcher_motion_timer = new ElapsedTime();

    // PID TUNING COEFFICIENTS
    @Config
    public static class LAUNCHER_PID_COEFFS {
        public static double kV = 0;
        public static double kA = 0;
        public static double kP = 0.0018;
        public static double kI = 0;
        public static double kD = 0.00013;
        public static double kF = 0.05;
        public static double KP_POS = 0;
    }
    public static double MAX_VEL = 1;
    public static double MAX_ACCEL = 1;
    public static double KP_POS = 0; // small correction kP for pose error
    double lastError = 0.0;
    double integral = 0.0;
    double lastPIDOutput = 0.0;

    private double degreesToTicks(double degrees) {
        return (degrees * (TICKS_PER_REV * GEAR_RATIO / 360.0));
    }

    private double ticksToDegrees(double ticks) {
//        double degrees = (ticks * 360.0 / (TICKS_PER_REV * GEAR_RATIO));
        double degrees = (ticks * 360.0 / (TICKS_PER_REV)) + launcherZeroPos;
//        if (degrees < 0) {
//            degrees += 360.0;
//        }
        return degrees;
    }

    public Launcher(HardwareMap hardwareMap) {
        launchMotor = hardwareMap.get(DcMotorEx.class, "launchMotor");
//        launchMotor.setDirection(DcMotorEx.Direction.REVERSE);
        launchMotor.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        launchMotor.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        launcherZeroPos = -180;

        targetAngle = REST_ANGLE;
    }

    public double getCurrentAngle() {
        return ticksToDegrees(launchMotor.getCurrentPosition());
    }

    public double getCurrentTicks() {
        return launchMotor.getCurrentPosition();
    }

    public double getOutput() {
        return launchMotor.getPower();
    }

    public void toggleShoot() {
        shootToggle = true;
    }

    public void spit() {
        launch_state = LAUNCH_STATE.SPIT;
        shootToggle = false;
    }

    public void toggleFlowerPower() {
        shoot_power = FLOWER_POWER;
    }

    public void toggleShootPower() {
        shoot_power = SHOOT_POWER;
    }

    public void runLauncher(double elapsedTime) {
//        double targetPos = degreesToTicks(REST_ANGLE);
        double currentAngle = getCurrentAngle();
        double currentVel = ticksToDegrees(launchMotor.getVelocity());

        if (shootToggle) {
            if (shoot_power == FLOWER_POWER) {
                targetAngle = targetAngle + (FLOWER_ANGLE - SHOOT_POS_ANGLE);
                launch_state = LAUNCH_STATE.SHOOT;
            }
            else {
                targetAngle = currentAngle - (((currentAngle % 360.0) - SHOOT_POS_ANGLE));
                launch_state = LAUNCH_STATE.POSITION;
            }
            shootToggle = false;
        }

        if (targetAngle != lastTargetAngle) {
            profileStartAngle = currentAngle;
            profileTargetAngle = targetAngle;
            profileDir = Math.signum(targetAngle - currentAngle);
            launcher_motion_timer.reset();
            integral = 0;
            lastError = 0;
            lastTargetAngle = targetAngle;
        }

        double error = targetAngle - currentAngle;
        double pidOutput = 1.0;

        if (launch_state == LAUNCH_STATE.SHOOT && !(shoot_power == FLOWER_POWER) && error > 270) {
            launchMotor.setPower(shoot_power);
        }
        else if (launch_state != LAUNCH_STATE.SPIT) {
            launchMotor.setPower(-0.5);
            launch_state = LAUNCH_STATE.REST;
            lastPIDOutput = -0.5;
        }
        else {
            // Motion profile
            double dist = Math.abs(profileTargetAngle - profileStartAngle);
            double[] profile = Shooter.motion_profile(MAX_ACCEL, MAX_VEL, dist, launcher_motion_timer.seconds());

            // predicted values
            double refAngle = profileStartAngle + profileDir * profile[0];
            double refVel = profileDir * profile[1];
            double refAcc = profileDir * profile[2];

            // errors
            double velError = refVel - currentVel;
            double posError = refAngle - currentAngle;

            integral += error * elapsedTime;
            final double I_ZONE = 40;        // only integrate within 40 deg of target
            final double I_MAX_POWER = 0.2;  // integral only contributes max of 0.2 power

            if (Math.abs(error) < I_ZONE) {
                integral += error * elapsedTime;
                if (LAUNCHER_PID_COEFFS.kI != 0) {
                    double lim = I_MAX_POWER / LAUNCHER_PID_COEFFS.kI;
                    integral = Math.max(-lim, Math.min(lim, integral));
                }
            } else {
                integral = 0;
            }

            double derivative = (velError - lastError) / elapsedTime;
            lastError = velError;

            pidOutput = LAUNCHER_PID_COEFFS.kV * refVel +
                    LAUNCHER_PID_COEFFS.kA * refAcc +
                    LAUNCHER_PID_COEFFS.kP * velError +
                    LAUNCHER_PID_COEFFS.kI * integral +
                    LAUNCHER_PID_COEFFS.kD * derivative +
                    LAUNCHER_PID_COEFFS.KP_POS * posError;

            pidOutput += LAUNCHER_PID_COEFFS.kF * Math.sin(Math.toRadians(currentAngle % 360));

            if (SLEW_PID) {
                pidOutput = Helpers.slew(SLEW_RATE, pidOutput, lastPIDOutput);
            }
            lastPIDOutput = pidOutput;

            launchMotor.setPower(pidOutput);
        }

        if (Math.abs(error) < ANGLE_TOLERANCE_TO_SHOOT && launch_state == LAUNCH_STATE.POSITION) {
            launch_state = LAUNCH_STATE.SHOOT;
            targetAngle = targetAngle + (SHOOT_ANGLE - SHOOT_POS_ANGLE);
//            targetAngle = currentAngle + (SHOOT_ANGLE - (currentAngle % (360*3)));
        }
        else if (Math.abs(error) < ANGLE_TOLERANCE_TO_REST && launch_state == LAUNCH_STATE.SHOOT) {
            launch_state = LAUNCH_STATE.REST;
            targetAngle = currentAngle + (REST_ANGLE - (currentAngle % 360));
        }



//        TelemetryManager panelsTelemetry = Panels.getTelemetry();
//        panelsTelemetry.debug("Init was ran!");
//        panelsTelemetry.graph("Current Angle", currentAngle);
//        panelsTelemetry.graph("Target Angle", targetAngle);
//        panelsTelemetry.graph("Error", error);
//        panelsTelemetry.graph("PIDOutput", pidOutput);
//        panelsTelemetry.update();
    }

    public void shiftZeroPositionForward() {
        launcherZeroPos += NUDGE_ZERO_AMOUNT;
    }

    public void shiftZeroPositionBackward() {
        launcherZeroPos -= NUDGE_ZERO_AMOUNT;
    }

    public void launcherFoward() {
        launchMotor.setPower(1);
    }

    public void launcherForwardSlow() {
        launchMotor.setPower(0.25);
    }

    public void rest() {
        launchMotor.setPower(0);
    }

    public void shootState() {
        launch_state = LAUNCH_STATE.SHOOT;
        targetAngle = targetAngle + (SHOOT_ANGLE - SHOOT_POS_ANGLE);
    }

}
