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

    static boolean SLEW_PID = true;
    static double SLEW_RATE = 0.15;
    final double REST_ANGLE = 260.0;
    final double SHOOT_POS_ANGLE = 50.0;
    final double SHOOT_ANGLE = 360 * 2.5;
    final double GEAR_RATIO = 10.0 / 24.0;
    final double TICKS_PER_REV = 220.0; // gobilda - 357.7?
    double ANGLE_TOLERANCE_TO_SHOOT = 20;
    double ANGLE_TOLERANCE_TO_REST = 30;
    double shoot_power = 1;

    boolean shootToggle = false;
    enum STATE {REST, POSITION, SHOOT};
    STATE launch_state = STATE.REST;
    double targetAngle = 0;
    enum typeShoot {FLOWER, SHOOT};
    typeShoot typeShootToggle = typeShoot.SHOOT;
    final double FLOWER_POWER = 0.2;
    final double SHOOT_POWER = 1;
    TelemetryManager panelsTelemetry = Panels.getTelemetry();

    @Config
    public static class PID_COEFFS {
        public static double kP = 0.0018;
        public static double kI = 0;
        public static double kD = 0.00013;
        public static double kF = 0.05;
    }
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

    public boolean isFlowerMode() {
        if (typeShootToggle == typeShoot.FLOWER) {
            return true;
        }
        return false;
    }

    public void toggleShootType() {
        if (typeShootToggle == typeShoot.SHOOT) {
            typeShootToggle = typeShoot.FLOWER;
            shoot_power = FLOWER_POWER;
        }
        else {
            typeShootToggle = typeShoot.SHOOT;
            shoot_power = SHOOT_POWER;
        }
    }

//    public void toggleFlower() {
//        typeShootToggle = typeShoot.FLOWER;
//        shoot_power = FLOWER_POWER;
//    }

    public void runLauncher(double elapsedTime) {
//        double targetPos = degreesToTicks(REST_ANGLE);
        double currentAngle = getCurrentAngle();

        if (shootToggle) {
            targetAngle = currentAngle - (((currentAngle % 360.0) - SHOOT_POS_ANGLE));
            launch_state = STATE.POSITION;
            shootToggle = false;
        }

        double error = targetAngle - currentAngle;
        double pidOutput = 1.0;

        if (launch_state == STATE.SHOOT && error > 270) {
            launchMotor.setPower(shoot_power);
        }
        else {
            integral += error * elapsedTime;
            final double I_ZONE = 40;        // only integrate within 40 deg of target
            final double I_MAX_POWER = 0.2;  // integral can contribute at most 0.2 power

            if (Math.abs(error) < I_ZONE) {
                integral += error * elapsedTime;
                if (PID_COEFFS.kI != 0) {
                    double lim = I_MAX_POWER / PID_COEFFS.kI;
                    integral = Math.max(-lim, Math.min(lim, integral));
                }
            } else {
                integral = 0;
            }

            double derivative = (error - lastError) / elapsedTime;
            lastError = error;

            pidOutput = PID_COEFFS.kP * error +
                    PID_COEFFS.kI * integral +
                    PID_COEFFS.kD * derivative;

            pidOutput += PID_COEFFS.kF * Math.sin(Math.toRadians(currentAngle % 360));

            if (SLEW_PID) {
                pidOutput = Helpers.slew(SLEW_RATE, pidOutput, lastPIDOutput);
            }
            lastPIDOutput = pidOutput;

            launchMotor.setPower(pidOutput);
        }

        if (Math.abs(error) < ANGLE_TOLERANCE_TO_SHOOT && launch_state == STATE.POSITION) {
            launch_state = STATE.SHOOT;
            targetAngle = targetAngle + (SHOOT_ANGLE - SHOOT_POS_ANGLE);
//            targetAngle = currentAngle + (SHOOT_ANGLE - (currentAngle % (360*3)));
        }
        else if (Math.abs(error) < ANGLE_TOLERANCE_TO_REST && launch_state == STATE.SHOOT) {
            launch_state = STATE.REST;
            targetAngle = currentAngle + (REST_ANGLE - (currentAngle % 360));
        }

        TelemetryPacket packet = new TelemetryPacket();

        packet.put("Launch State", launch_state.toString());
        packet.put("Launch power", shoot_power);
//        packet.put("Current Angle", currentAngle);
//        packet.put("Target Angle", targetAngle);
//        packet.put("Error", error);
//        packet.put("PID Output", pidOutput);

        packet.put("kP", PID_COEFFS.kP);
        packet.put("kI", PID_COEFFS.kI);
        packet.put("kD", PID_COEFFS.kD);
        packet.put("kF", PID_COEFFS.kF);

        FtcDashboard.getInstance().sendTelemetryPacket(packet);

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
        launch_state = STATE.SHOOT;
        targetAngle = targetAngle + (SHOOT_ANGLE - SHOOT_POS_ANGLE);
    }

    public void spit() {
        launchMotor.setPower(-1);
    }
}
