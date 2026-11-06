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
public class TeleOP extends LinearOpMode {
    Robot robot;

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
        robot = new Robot(hardwareMap);

        // Retrieve the IMU from the hardware map
        imu = hardwareMap.get(IMU.class, "imu");
        // Adjust the orientation parameters to match your robot <------------------------------------------------------- IMPORTANT
        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.LEFT,
                RevHubOrientationOnRobot.UsbFacingDirection.UP));
        // Without this, the REV Hub's orientation is assumed to be logo up / USB forward
        imu.initialize(parameters);

        waitForStart();

        robot.initialize();

        if (isStopRequested()) return;

        while (opModeIsActive()) {
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

//            frontLeftMotor.setPower(frontLeftPower);
//            backLeftMotor.setPower(backLeftPower);
//            frontRightMotor.setPower(frontRightPower);
//            backRightMotor.setPower(backRightPower);

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

            if (gamepad1.aWasPressed() || gamepad2.aWasPressed()) {
                intake.toggle();
            }
//            if (gamepad2.aWasPressed()) {
//                intake.overrideIntake();
//            }
            if (gamepad1.x) {
                intake.spit();
            }

            if (gamepad2.xWasPressed()) {
                flower.toggleFlowerDescorer();
            }

            if (gamepad1.dpadUpWasPressed()) {
                robot.changeHoodPos(Shooter.HOOD_STATE.HIGH);
            }
            if (gamepad1.dpadDownWasPressed()) {
                robot.changeHoodPos(Shooter.HOOD_STATE.LOW);
            }
//            if (gamepad1.dpadLeftWasPressed()) {
////                shooter.hood.hoodShoot();
//                shooter.hood.hoodShootMidHigh();
//            }
//            if (gamepad1.dpadRightWasPressed()) {
////                shooter.hood.hoodFlower();
//                shooter.hood.hoodShootMidLow();
//            }
            if (gamepad2.dpadUpWasPressed()) {
                robot.changeHoodPos(Shooter.HOOD_STATE.INC);
            }
            if (gamepad2.dpadDownWasPressed()) {
                robot.changeHoodPos(Shooter.HOOD_STATE.DEC);
            }

            if (gamepad2.right_trigger > 0.5) {
                elevator.runForward(gamepad2.right_trigger);
            }
            else if (gamepad2.left_trigger > 0.5) {
                elevator.runBackward(gamepad2.left_trigger);
            }
            else {
                elevator.rest();
            }

            if (gamepad1.rightBumperWasPressed() || gamepad2.rightBumperWasPressed()) {
                elevator.toggleTopPos();
            }
            if (gamepad1.leftBumperWasPressed() || gamepad2.leftBumperWasPressed()) {
                elevator.toggleBottomPos();
            }

            // flower
            if (gamepad1.yWasPressed() || gamepad2.yWasPressed()) {
                robot.toggleFlower();
            }

            robot.runMechanisms();

            telemetry.addData("Flower state", shooter.launcher.isFlowerMode());
            telemetry.addData("Hood pos", shooter.hood.getPos());
            telemetry.update();

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
