package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;

import org.firstinspires.ftc.teamcode.Intake;
import org.firstinspires.ftc.teamcode.Shooter.Shooter;

@TeleOp
public class RobotOP extends LinearOpMode {
    Intake intake;
    Shooter shooter;
    DcMotor frontLeftMotor;
    DcMotor backLeftMotor;
    DcMotor frontRightMotor;
    DcMotor backRightMotor;
    IMU imu;

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

        frontLeftMotor = hardwareMap.dcMotor.get("frontLeft");
        backLeftMotor = hardwareMap.dcMotor.get("backLeft");
        frontRightMotor = hardwareMap.dcMotor.get("frontRight");
        backRightMotor = hardwareMap.dcMotor.get("backRight");

        // Reverse some of the drive motors depending on physical setup
        frontLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        backLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        // Retrieve the IMU from the hardware map
        imu = hardwareMap.get(IMU.class, "imu");
        // Adjust the orientation parameters to match your robot <------------------------------------------------------- IMPORTANT
        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.LEFT,
                RevHubOrientationOnRobot.UsbFacingDirection.UP));
        // Without this, the REV Hub's orientation is assumed to be logo up / USB forward
        imu.initialize(parameters);

        waitForStart();

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
            frontLeftMotor.setPower(-gamepad1.left_stick_y);
            backLeftMotor.setPower(-gamepad1.left_stick_y);
            frontRightMotor.setPower(-gamepad1.right_stick_y);
            backRightMotor.setPower(-gamepad1.right_stick_y);

//            telemetry.addData("vals", "%4.2f, %4.2f, %4.2f", y, x, rx);
//            telemetry.addData("Front left/Right", "%4.2f, %4.2f", frontLeftPower, frontRightPower);
//            telemetry.addData("Back  left/Right", "%4.2f, %4.2f", backLeftPower, backRightPower);


            // Testing
//            if (gamepad1.aWasPressed() || gamepad2.aWasPressed()) {
//                rightFlywheel.setPower(1);
//                leftFlywheel.setPower(1);
//            }

            if (gamepad1.b) {
                shooter.launcher.shootBall();
            }
            else if (gamepad1.a) {
                shooter.launcher.flowerBall();
            }
            else {
                shooter.launcher.rest();
            }

            if (gamepad1.yWasPressed()) {
                intake.runIntakeForward();
            }
            else if (gamepad1.xWasPressed()) {
                intake.runIntakeBackward();
            }
            else {
                intake.rest();
            }

            if (gamepad1.dpadUpWasPressed()) {
                shooter.hood.hoodShoot();
            }
            if (gamepad1.dpadDownWasPressed()) {
                shooter.hood.hoodFlower();
            }

            telemetry.update();
        }
    }
}
