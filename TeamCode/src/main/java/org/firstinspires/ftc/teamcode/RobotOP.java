package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.Intake;
import org.firstinspires.ftc.teamcode.Shooter.Hood;
import org.firstinspires.ftc.teamcode.Shooter.Launcher;
import org.firstinspires.ftc.teamcode.Shooter.Shooter;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;

@TeleOp @Config
public class RobotOP extends LinearOpMode {
    Intake intake;
    Shooter shooter;
    Elevator elevator;
    Flower flower;
    DcMotor frontLeftMotor;
    DcMotor backLeftMotor;
    DcMotor frontRightMotor;
    DcMotor backRightMotor;

    IMU imu;
    ElapsedTime timer = new ElapsedTime();
    double timeForLauncherStart = 100;

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
        shooter = new Shooter(hardwareMap);
        intake = new Intake(hardwareMap);
        elevator = new Elevator(hardwareMap);
        flower = new Flower(hardwareMap);

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
        timer.reset();

        elevator.toggleTopPos();
        flower.openServo();

        if (isStopRequested()) return;

        while (opModeIsActive()) {
            double dt = timer.seconds();
            timer.reset();

            // Take whichever value is the most drastic change to use from either controller
            double y = calcLargestChange(-gamepad1.left_stick_y, -gamepad2.left_stick_y); // Y stick values are reported as inverted by the controller
            double x = calcLargestChange(gamepad1.left_stick_x, gamepad2.left_stick_x);
            double rx = calcLargestChange(gamepad1.right_stick_x, gamepad2.right_stick_x);

            // This button choice was made so that it is hard to hit on accident,
            // it can be freely changed based on preference.
            // The equivalent button is start on Xbox-style controllers.
            if (gamepad1.options) {
                imu.initialize(parameters);
                imu.resetYaw();
            }

            double botHeading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);

            // Rotate the movement direction counter to the bot's rotation
            double rotX = x * Math.cos(-botHeading) - y * Math.sin(-botHeading);
            double rotY = x * Math.sin(-botHeading) + y * Math.cos(-botHeading);

            rotX = rotX * 1.1;  // Counteract imperfect strafing

            // Denominator is the largest motor power (absolute value) or 1
            // This ensures all the powers maintain the same ratio,
            // but only if at least one is out of the range [-1, 1]
            double denominator = Math.max(Math.abs(rotY) + Math.abs(rotX) + Math.abs(rx), 1);
            double frontLeftPower = (rotY + rotX + rx) / denominator;
            double backLeftPower = (rotY - rotX + rx) / denominator;
            double frontRightPower = (rotY - rotX - rx) / denominator;
            double backRightPower = (rotY + rotX - rx) / denominator;

//            telemetry.addData("vals", "%4.2f, %4.2f, %4.2f", y, x, rx);
//            telemetry.addData("Front left/Right", "%4.2f, %4.2f", frontLeftPower, frontRightPower);
//            telemetry.addData("Back  left/Right", "%4.2f, %4.2f", backLeftPower, backRightPower);

            frontLeftMotor.setPower(frontLeftPower);
            backLeftMotor.setPower(backLeftPower);
            frontRightMotor.setPower(frontRightPower);
            backRightMotor.setPower(backRightPower);

            // Delay for launcher to start to give time to flower servo
            if (timeForLauncherStart <= 0) {
                shooter.launcher.runLauncher(dt);
            }
            else {
                timeForLauncherStart -= 2;
            }

            if (gamepad1.bWasPressed() || gamepad2.bWasPressed()) {
                shooter.launcher.toggleShoot();
            }
            if (gamepad2.dpadRightWasPressed()) {
                shooter.launcher.shiftZeroPositionForward();
            }
            if (gamepad2.dpadLeftWasPressed()) {
                shooter.launcher.shiftZeroPositionBackward();
            }
//            telemetry.addData("Launch state: ", shooter.launcher.getCurrentState());

            if (gamepad2.backWasPressed()) {
                shooter.launcher.shootState();
            }

            intake.runIntake();
            if (gamepad1.aWasPressed()) {
                intake.toggle();
            }
            if (gamepad2.x) {
                intake.spit(); // spit out balls in case stuck
            }

            if (gamepad1.dpadUpWasPressed()) {
//                shooter.hood.hoodShoot();
                shooter.hood.hoodShootHigh();
            }
            if (gamepad1.dpadDownWasPressed()) {
//                shooter.hood.hoodFlower();
                shooter.hood.hoodShootLow();
            }
            if (gamepad2.dpadUpWasPressed()) {
                shooter.hood.incPosition();
            }
            if (gamepad2.dpadDownWasPressed()) {
                shooter.hood.decPosition();
            }
            shooter.hood.updateHood();

            if (gamepad2.right_trigger > 0.5) {
                elevator.runForward(gamepad2.right_trigger);
            }
            else if (gamepad2.left_trigger > 0.5) {
                elevator.runBackward(gamepad2.left_trigger);
            }
            else {
                elevator.rest();
            }

            if (gamepad1.rightBumperWasPressed()) {
                elevator.toggleTopPos();
            }
            if (gamepad1.leftBumperWasPressed()) {
                elevator.toggleBottomPos();
            }
            elevator.runElevator(dt);

            // flower
            if (gamepad1.yWasPressed() || gamepad2.yWasPressed()) {
                shooter.launcher.toggleShootType();
                if (shooter.launcher.isFlowerMode()) {
                    elevator.toggleFlowerPos();
                    shooter.hood.hoodFlower();
                }
            }

//            if (gamepad1.dpad_right) {
//                shooter.launcher.launcherFoward();
//            }
//            if (gamepad1.dpad_left) {
//                shooter.launcher.launcherForwardSlow();
//            }

            TelemetryPacket packet = new TelemetryPacket();
            packet.put("launcher angle", shooter.launcher.getCurrentAngle());
            packet.put("launcher ticks", shooter.launcher.getCurrentTicks());
//            packet.put("launch angle", shooter.launcher.getCurrentAngle());
//            packet.put("launch output", shooter.launcher.getOutput());
//            packet.put("kP", shooter.launcher.PID_COEFFS.kP);
//            packet.put("kI", shooter.launcher.PID_COEFFS.kI);
//            packet.put("kD", shooter.launcher.PID_COEFFS.kD);
//            packet.put("kF", shooter.launcher.PID_COEFFS.kF);
            FtcDashboard.getInstance().sendTelemetryPacket(packet);
        }
    }
}
