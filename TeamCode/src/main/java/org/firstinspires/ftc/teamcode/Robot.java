package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.pedropathing.math.Pose;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Shooter.Shooter;

@Config
public class Robot {
    // Mechanisms
    Intake intake;
    Shooter shooter;
    Flower flower;
    DcMotor frontLeftMotor;
    DcMotor backLeftMotor;
    DcMotor frontRightMotor;
    DcMotor backRightMotor;
    Servo ledIndicator;

    double timeForLauncherStart = 100;

    // Robot pose
    public static Pose robotPose;

    ElapsedTime timer = new ElapsedTime();

    public Robot(HardwareMap hardwareMap) {
        shooter = new Shooter(hardwareMap);
        intake = new Intake(hardwareMap);
        flower = new Flower(hardwareMap);

        ledIndicator = hardwareMap.get(Servo.class, "led");

        frontLeftMotor = hardwareMap.dcMotor.get("frontLeft");
        backLeftMotor = hardwareMap.dcMotor.get("backLeft");
        frontRightMotor = hardwareMap.dcMotor.get("frontRight");
        backRightMotor = hardwareMap.dcMotor.get("backRight");

        // Reverse some of the drive motors depending on physical setup
        frontLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        backLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        timer.reset();
    }

    public void initialize() {
        shooter.initialize();
        flower.extendFlowerServo();
        flower.toggleFlowerDescorer();
    }

    public void toggleFlower() {
        shooter.toggleFlowerMode();

        // Set LED indicator light
        if(shooter.isFlowerMode()) {
            ledIndicator.setPosition(0.5); // green
        } else {
            ledIndicator.setPosition(0.28); // red
        }
    }

    public void spitAll() {
        shooter.launcher.spit();
        intake.spit();
    }

    public void runMechanisms() {
        double dt = timer.seconds();
        timer.reset();

        // Delay for launcher to start to give time to flower servo
        if (timeForLauncherStart <= 0) {
            shooter.launcher.runLauncher(dt);
        }
        else {
            timeForLauncherStart -= 2;
        }

        intake.runIntake();
        shooter.updateShooter(dt);

        TelemetryPacket packet = new TelemetryPacket();

//        packet.put("Launch LAUNCH_STATE", Shooter.launcher.launch_state.toString());
//        packet.put("Launch power", shoot_power);
//        packet.put("Launch shoot toggle", launchTypeToggle.toString());
//        packet.put("Current Angle", currentAngle);
//        packet.put("Target Angle", targetAngle);
//        packet.put("Error", error);
//        packet.put("PID Output", pidOutput);

//        packet.put("kP", Launcher.PID_COEFFS.kP);
//        packet.put("kI", Launcher.PID_COEFFS.kI);
//        packet.put("kD", Launcher.PID_COEFFS.kD);
//        packet.put("kF", Launcher.PID_COEFFS.kF);

        FtcDashboard.getInstance().sendTelemetryPacket(packet);
    }
}
