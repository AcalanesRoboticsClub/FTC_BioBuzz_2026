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
    final double REST_ANGLE = 270.0;
    final double SHOOT_POS_ANGLE = 25.0;
    final double SHOOT_ANGLE = 360 * 1.5;
    final double GEAR_RATIO = 10.0 / 24.0;
    final double TICKS_PER_REV = 220.0; // gobilda - 357.7?
    double ANGLE_TOLERANCE = 2;

    boolean shootToggle = false;
    enum STATE {REST, POSITION, SHOOT};
    STATE launch_state = STATE.REST;
    double targetAngle = 0;
    enum typeShoot {FLOWER, SHOOT};
    typeShoot typeShootToggle = typeShoot.SHOOT;
    final double FLOWER_POWER = 0.75;
    final double SHOOT_POWER = 1;
    TelemetryManager panelsTelemetry = Panels.getTelemetry();

    @Config
    public static class PID_COEFFS {
        public static double kP = 0.0;
        public static double kI = 0;
        public static double kD = 0;
        public static double kF = 0;
    }
    double lastError = 0.0;
    double integral = 0.0;

    private double degreesToTicks(double degrees) {
        return (degrees * (TICKS_PER_REV * GEAR_RATIO / 360.0));
    }

    private double ticksToDegrees(double ticks) {
//        double degrees = (ticks * 360.0 / (TICKS_PER_REV * GEAR_RATIO));
        double degrees = (ticks * 360.0 / (TICKS_PER_REV));
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

    }

    public double getCurrentAngle() {
        return ticksToDegrees(launchMotor.getCurrentPosition());
    }

    public double getCurrentTicks() {
        return launchMotor.getCurrentPosition();
    }

    public STATE getCurrentState() {
        return launch_state;
    }

    public double getOutput() {
        return launchMotor.getPower();
    }

    public void toggleShoot() {
        shootToggle = true;
    }

    public void toggleShootType() {
        if (typeShootToggle == typeShoot.SHOOT) {
            typeShootToggle = typeShoot.FLOWER;
        }
        else {
            typeShootToggle = typeShoot.SHOOT;;
        }
    }

    public void runLauncher(double elapsedTime) {
//        double targetPos = degreesToTicks(REST_ANGLE);
        double currentAngle = getCurrentAngle();

        if (shootToggle) {
            targetAngle = currentAngle - (((currentAngle % 360.0) - SHOOT_POS_ANGLE));
            launch_state = STATE.POSITION;
        }
        else if (launch_state == STATE.SHOOT) {
            targetAngle = currentAngle + (SHOOT_ANGLE - (currentAngle % 640));
        }
        else if (launch_state == STATE.REST) {
            targetAngle = currentAngle + (REST_ANGLE - (currentAngle % 360));
        }

        double error = targetAngle - currentAngle;

        integral += error * elapsedTime;
        double derivative = (error - lastError) / elapsedTime;
        lastError = error;

        double pidOutput = PID_COEFFS.kP * error +
                PID_COEFFS.kI * integral +
                PID_COEFFS.kD * derivative;

        pidOutput += PID_COEFFS.kF * Math.sin(Math.toRadians(currentAngle % 360));

        launchMotor.setPower(pidOutput);

        if (Math.abs(error) < ANGLE_TOLERANCE && launch_state == STATE.POSITION) {
            launch_state = STATE.SHOOT;
            shootToggle = false;
        }
        else if (Math.abs(error) < ANGLE_TOLERANCE && launch_state == STATE.SHOOT) {
            launch_state = STATE.REST;
        }

        TelemetryPacket packet = new TelemetryPacket();

        packet.put("Launch State", launch_state.toString());
        packet.put("Current Angle", currentAngle);
        packet.put("Target Angle", targetAngle);
        packet.put("Error", error);
        packet.put("PID Output", pidOutput);

        packet.put("kP", PID_COEFFS.kP);
        packet.put("kI", PID_COEFFS.kI);
        packet.put("kD", PID_COEFFS.kD);
        packet.put("kF", PID_COEFFS.kF);

//        FtcDashboard.getInstance().sendTelemetryPacket(packet);

//        panelsTelemetry.debug("Init was ran!");
//        panelsTelemetry.graph("Current Angle", currentAngle);
//        panelsTelemetry.graph("Target Angle", targetAngle);
//        panelsTelemetry.graph("Error", error);
//        panelsTelemetry.graph("PIDOutput", pidOutput);
//        panelsTelemetry.update();
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

    public void spit() {
        launchMotor.setPower(-1);
    }
}
