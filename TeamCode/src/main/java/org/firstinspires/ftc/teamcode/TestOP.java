// Written by AcaBots FTC team 24689 for the 2025-26 DECODE Season

/*
-------------------- CONTROL SCHEME - CONTROLLER 1 --------------------
Buttons:
    A: Toggle intake, transfer, and riser belts
    START: Reset IMU
    BACK: Reverse intake, transfer, and riser belts + turret intake kicker wheel


D-Pad:
    UP: Close/lobbing preset
    DOWN: Far launch zone preset
    RIGHT: Middle launch zone preset, low angle
    LEFT: Middle launch zone preset, high angle

Triggers:
    RT: Turret rotate right
    LT: Turret rotate left

Shoulder Buttons:
    RB: Turret flywheel
    LB: Turret intake kicker wheel

Joysticks:
    Right: Relative Chassis Rotation
    Left: Absolute Chassis Strafe based on orientation when START button is pressed

-------------------- CONTROL SCHEME - CONTROLLER 2 --------------------
Same as controller 1, except turret rotation is disabled
 */

package org.firstinspires.ftc.teamcode;
import android.util.Size;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.Servo;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
//import com.acmerobotics.dashboard.telemetry.TelemetryPacket;


import java.util.List;

@TeleOp
public class TestOP extends LinearOpMode{
//    int FLYWHEEL_ROTATE_MAX = 1450;
//    int FLYWHEEL_ROTATE_MIN = -2050;
//    boolean intakeToggle = false;
//    double flywheelVelocitySet = 0;
    DcMotor frontLeftMotor; // 1
    DcMotor backLeftMotor; // 0
    DcMotor frontRightMotor; // 1 (expansion)
    DcMotor backRightMotor; // 0 (expansion)
    CRServo testServo; // 0
//    DcMotor leftFlywheel; // 1
//    DcMotor rightFlywheel; // 0
    DcMotorEx adjustFlywheel; // 2
    IMU imu;
    double power;

    private double calcLargestChange(double a, double b) {
        // Return the value of the greatest absolute value of either a or b. Used for dual controller input
        if(Math.abs(b) > Math.abs(a)) {
            return b;
        } else {
            return a;
        }
    }

    public double expo(double input, double exponent) { // for scaled non linear throttle curve driving
        return Math.signum(input) * Math.pow(Math.abs(input), exponent);
    }

    @Override
    public void runOpMode() throws InterruptedException {
//        TelemetryPacket packet = new TelemetryPacket();

        // Hardware Definitions. Must match names setup in robot configuration in the driver hub. config is created and selected selected with driver hub menu
        // Drive Motors
//        frontLeftMotor = hardwareMap.dcMotor.get("frontLeftMotor");
//        backLeftMotor = hardwareMap.dcMotor.get("backLeftMotor");
//        frontRightMotor = hardwareMap.dcMotor.get("frontRightMotor");
//        backRightMotor = hardwareMap.dcMotor.get("backRightMotor");
//        testServo = hardwareMap.get(CRServo.class, "testServo");
//        beltLeft.setDirection(CRServo.Direction.REVERSE);

        // Flywheel Motor and Rotation
//        flywheelRotateMotor = hardwareMap.dcMotor.get("rotatShot");
//        flywheelRotateMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        flywheelRotateMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//        flywheelRotateMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

//        leftFlywheel = hardwareMap.dcMotor.get("leftFly");
//        rightFlywheel = hardwareMap.dcMotor.get("rightFly");
//        rightFlywheel.setDirection(DcMotorSimple.Direction.REVERSE);
        adjustFlywheel = hardwareMap.get(DcMotorEx.class, "adjusFly");
//        testMotor = hardwareMap.dcMotor.get("testMotor");

        // Reverse some of the drive motors depending on physical setup
//        frontLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
//        backLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        // Retrieve the IMU from the hardware map
        imu = hardwareMap.get(IMU.class, "imu");
        // Adjust the orientation parameters to match your robot <------------------------------------------------------- IMPORTANT
        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.LEFT,
                RevHubOrientationOnRobot.UsbFacingDirection.UP));
        // Without this, the REV Hub's orientation is assumed to be logo up / USB forward
        imu.initialize(parameters);

        waitForStart();

        power = 0.5;

        if (isStopRequested()) return;

        while (opModeIsActive()) {

            // ===== Adjustable expo values =====
            double driveExpo = 1.5;   // 1.0 = linear, 2.0 = quadratic, 3.0 = very soft center
            double turnExpo  = 1.5;

            // ===== Read & scale driver input =====
            double x  = expo(calcLargestChange(gamepad1.left_stick_x, gamepad2.left_stick_x), driveExpo);
            double y  = expo(calcLargestChange(-gamepad1.left_stick_y, -gamepad2.left_stick_y), driveExpo); // negative = forward
            double rx = expo(calcLargestChange(gamepad1.right_stick_x, gamepad2.right_stick_x), turnExpo);

            // ===== Field-centric math =====
            double botHeading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);

            // Rotate the movement direction counter to the bot's rotation
            double rotX = x * Math.cos(botHeading) - y * Math.sin(botHeading);
            double rotY = x * Math.sin(botHeading) + y * Math.cos(botHeading);

            rotX *= 1.1;  // Counteract imperfect strafing

            // Denominator is the largest motor power (absolute value) or 1
            double denominator = Math.max(Math.abs(rotY) + Math.abs(rotX) + Math.abs(rx), 1);

            double frontLeftPower  = (rotY + rotX + rx) / denominator;
            double backLeftPower   = (rotY - rotX + rx) / denominator;
            double frontRightPower = (rotY - rotX - rx) / denominator;
            double backRightPower  = (rotY + rotX - rx) / denominator;

            // ===== Normalize =====
            double max = Math.max(Math.abs(frontLeftPower), Math.abs(frontRightPower));
            max = Math.max(max, Math.abs(backLeftPower));
            max = Math.max(max, Math.abs(backRightPower));

            if (max > 1.0) {
                frontLeftPower  /= max;
                frontRightPower /= max;
                backLeftPower   /= max;
                backRightPower  /= max;
            }

            // ===== Apply power =====
//            frontLeftMotor.setPower(frontLeftPower);
//            backLeftMotor.setPower(backLeftPower);
//            frontRightMotor.setPower(frontRightPower);
//            backRightMotor.setPower(backRightPower);

//            telemetry.addData("vals", "%4.2f, %4.2f, %4.2f", y, x, rx);
//            telemetry.addData("Front left/Right", "%4.2f, %4.2f", frontLeftPower, frontRightPower);
//            telemetry.addData("Back  left/Right", "%4.2f, %4.2f", backLeftPower, backRightPower);


            // Testing
//            if (gamepad1.aWasPressed() || gamepad2.aWasPressed()) {
//                rightFlywheel.setPower(1);
//                leftFlywheel.setPower(1);
//            }

            if (gamepad1.b) {
//                rightFlywheel.setPower(-1);
//                leftFlywheel.setPower(-1);
                adjustFlywheel.setPower(-power);
            }
            else if (gamepad1.x) {
//                rightFlywheel.setPower(1);
//                leftFlywheel.setPower(1);
                adjustFlywheel.setPower(power);
            }
            else {
                adjustFlywheel.setPower(0);
            }

            if (gamepad1.dpadUpWasPressed()) {
                power += 0.1;
            }
            else if (gamepad1.dpadDownWasPressed()) {
                power -= 0.1;
            }

            telemetry.addData("Motor current (A)", adjustFlywheel.getCurrent(CurrentUnit.AMPS));



//            if (gamepad1.xWasPressed() || gamepad2.xWasPressed()) {
//                testMotor.setPower(1);
//            }
//
//            if (gamepad1.yWasPressed() || gamepad2.yWasPressed()) {
//                testMotor.setPower(-1);
//            }

            telemetry.update();
        }
    }
}