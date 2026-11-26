package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.pedropathing.drivetrain.DrivePowers;
import com.pedropathing.follower.Follower;
import com.pedropathing.follower.ManualDrive;
import com.pedropathing.math.Pose;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.teamcode.Shooter.Shooter;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;

@TeleOp
@Config
public class TeleOP extends LinearOpMode {
    Robot robot;

    IMU imu; // TODO: Can remove?

    private Follower follower;

    private enum DRIVE {JOYSTICK, HOLD, REST, AUTO_PATH};

    DRIVE driveMode;

    private double calcLargestChange(double a, double b) {
        // Return the value of the greatest absolute value of either a or b. Used for dual controller input
        if(Math.abs(b) > Math.abs(a)) {
            return b;
        } else {
            return a;
        }
    }

    public double expo(double input, double exponent) { // for scaled non-linear throttle curve driving
        return Math.signum(input) * Math.pow(Math.abs(input), exponent);
    }

    private double deadbandExclude(double input, double deadband) {
        if(Math.abs(input) > deadband) {
            return input;
        }
        return 0;
    }

    @Override
    public void runOpMode() throws InterruptedException {
        robot = new Robot(hardwareMap);
        follower = PedroConstants.create(hardwareMap);

        double teleHeadingOffset = 0;

        // Retrieve the IMU from the hardware map
        imu = hardwareMap.get(IMU.class, "imu");
        // Adjust the orientation parameters to match your robot
        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.LEFT,
                RevHubOrientationOnRobot.UsbFacingDirection.UP));
        // Without this, the REV Hub's orientation is assumed to be logo up / USB forward
        imu.initialize(parameters);

        waitForStart();

        driveMode = DRIVE.REST; // Default to resting with no motor power
        double timeLastJoystick = getRuntime(); // Used for timeout coasting before holding pose

        // Set desired pose to zero if no pose was stored in auto
        Pose desiredPose = (HandoffPoseStorage.robotPose != null) ? HandoffPoseStorage.robotPose : new Pose(0, 0, 0);
        follower.setPose(desiredPose);

        // Set alliance color to blue if it was not set during auto
        HandoffPoseStorage.alliance_color = (HandoffPoseStorage.alliance_color != null) ? HandoffPoseStorage.alliance_color : HandoffPoseStorage.ALLIANCE.BLUE;

        robot.deploy();

        if (isStopRequested()) return;

        while (opModeIsActive()) {
            // ---------------------------- Chassis Driving Logic --------------------------------
            final double JOYSTICK_DEADBAND = 0.03;
            final double COAST_SECONDS = 1.00;
            final double DRIVE_CURVE_EXP = 1.5;

            // If any joystick has moved
            boolean joystickInput = Math.abs(gamepad1.left_stick_y)  > JOYSTICK_DEADBAND
                                 || Math.abs(gamepad2.left_stick_y)  > JOYSTICK_DEADBAND
                                 || Math.abs(gamepad1.left_stick_x)  > JOYSTICK_DEADBAND
                                 || Math.abs(gamepad2.left_stick_x)  > JOYSTICK_DEADBAND
                                 || Math.abs(gamepad1.right_stick_x) > JOYSTICK_DEADBAND
                                 || Math.abs(gamepad2.right_stick_x) > JOYSTICK_DEADBAND;

            // Determine the driving mode
            if(joystickInput) {
                driveMode = DRIVE.JOYSTICK;
                timeLastJoystick = getRuntime();
            }
            else {
                if (timeLastJoystick + COAST_SECONDS > getRuntime()) { // Allow time after input for robot to settle
                    driveMode = DRIVE.REST;
                    desiredPose = follower.pose(); // ensure robot doesn't lurch after exiting rest
                }
                else { // Wait until coasting delay after joystick has elapsed
                    if(driveMode == DRIVE.JOYSTICK) { // if this is the first time not being in joystick drive mode, otherwise it holds last saved pos
                        desiredPose = follower.pose(); // save current pose as the pose to hold
                    }

                    driveMode = DRIVE.HOLD;
                }
            }


            // States for Chassis driving (drive mode)
            if (driveMode == DRIVE.REST) { // No power to the motors
                DrivePowers powers = DrivePowers.zero();
                follower.manual(powers);
            }
            else if (driveMode == DRIVE.HOLD) { // Hold the last known intentional pose
                follower.hold(desiredPose);
                telemetry.addData("Holding Pose: ", desiredPose);
            }
            else if (driveMode == DRIVE.JOYSTICK) { // Allow joysticks to move directly
                // Take whichever value is the most drastic change to use from either controller, ignore if a specific axis is very close to zero, then scale using a throttle curve
                double y =  expo(deadbandExclude(calcLargestChange(gamepad1.left_stick_y, gamepad2.left_stick_y),   JOYSTICK_DEADBAND), DRIVE_CURVE_EXP);
                double x =  expo(deadbandExclude(calcLargestChange(gamepad1.left_stick_x, gamepad2.left_stick_x),   JOYSTICK_DEADBAND), DRIVE_CURVE_EXP);
                double rx = expo(deadbandExclude(calcLargestChange(gamepad1.right_stick_x, gamepad2.right_stick_x), JOYSTICK_DEADBAND), DRIVE_CURVE_EXP);

                // Flip input if driver is on the other side of the field. Pedro coordinates assume red alliance
                if(HandoffPoseStorage.alliance_color == HandoffPoseStorage.ALLIANCE.BLUE) {
                    y *= -1;
                    x *= -1;
                    rx *= -1;
                }

                DrivePowers powers = ManualDrive.fieldCentric(y, x, rx, follower.pose().heading() - teleHeadingOffset);

                follower.manual(powers);

                telemetry.addData("Drive powers: ", powers);
            }
            // ---------------------------- End Chassis Driving Logic --------------------------------


            // START button on F310 controllers
            // Press to reset teleop heading, and hold X or Y to set alliance colors
            // X and Y buttons chose to not conflict with gamepad 1/2 config
            if (gamepad1.optionsWasReleased() || gamepad2.optionsWasReleased()) {
                teleHeadingOffset = follower.pose().heading();   // whichever way the robot faces now counts as "forward"

                if(gamepad1.x || gamepad2.x) { // If blue X button is also being held
                    HandoffPoseStorage.alliance_color = HandoffPoseStorage.ALLIANCE.BLUE;
                }
                else if(gamepad1.y || gamepad2.y) { // If Y button is also being held
                    HandoffPoseStorage.alliance_color = HandoffPoseStorage.ALLIANCE.RED;
                }
            }

            if (gamepad1.bWasPressed() || gamepad2.bWasPressed()) {
                robot.shooter.paddle.toggleShoot();
            }
//            if (gamepad2.dpadRightWasPressed()) {
//                robot.shooter.launcher.shiftZeroPositionForward();
//            }
//            if (gamepad2.dpadLeftWasPressed()) {
//                robot.shooter.launcher.shiftZeroPositionBackward();
//            }
//            telemetry.addData("Launch state: ", shooter.launcher.getCurrentState());

            if (gamepad2.backWasPressed()) {
                robot.shooter.paddle.shootState();
            }
            if (gamepad1.backWasPressed()) {
                robot.shooter.paddle.paddleFoward();
            }

            if (gamepad1.aWasPressed() || gamepad2.aWasPressed()) {
                robot.intake.toggle();
            }
//            if (gamepad2.aWasPressed()) {
//                intake.overrideIntake();
//            }
//            if (gamepad1.x || gamepad2.x) {
//                robot.intake.spit();
//            }
            if (gamepad1.xWasPressed() || gamepad2.xWasPressed()) {
                robot.shooter.paddle.toggleShoot();
            }
            if (gamepad1.back || gamepad2.back) {
                robot.shooter.paddle.spit();
            }

//            if (gamepad2.xWasPressed()) {
//                robot.flower.toggleFlowerDescorer();
//            }

            // presets, will eventually also change elevator/launcher
            if (gamepad1.dpadUpWasPressed()) {
                robot.shooter.changeHoodPos(Shooter.HOOD_STATE.HIGH);
            }
            if (gamepad1.dpadDownWasPressed()) {
                robot.shooter.changeHoodPos(Shooter.HOOD_STATE.LOW);
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
                robot.shooter.changeHoodPos(Shooter.HOOD_STATE.INC_SMALL);
            }
            if (gamepad2.dpadDownWasPressed()) {
                robot.shooter.changeHoodPos(Shooter.HOOD_STATE.DEC_SMALL);
            }
            if (gamepad2.dpadRightWasPressed()) {
                robot.shooter.changeHoodPos(Shooter.HOOD_STATE.INC);
            }
            if (gamepad2.dpadLeftWasPressed()) {
                robot.shooter.changeHoodPos(Shooter.HOOD_STATE.DEC);
            }

            // Count number of pollen and nectar (control scheme later)
//            if (gamepad2.dpadUpWasPressed()) {
//                robot.shooter.incPollen();
//            }
//            if (gamepad2.dpadDownWasPressed()) {
//                robot.shooter.decPollen();
//            }
//            if (gamepad2.dpadRightWasPressed()) {
//                robot.shooter.incNectar();
//            }
//            if (gamepad2.dpadLeftWasPressed()) {
//                robot.shooter.decNectar();
//            }

            if (gamepad2.right_trigger > 0.5) {
                robot.shooter.elevator.runForward(gamepad2.right_trigger);
            }
            else if (gamepad2.left_trigger > 0.5) {
                robot.shooter.elevator.runBackward(gamepad2.left_trigger);
            }
            else {
                robot.shooter.elevator.rest();
            }

            if (gamepad1.rightBumperWasPressed() || gamepad2.rightBumperWasPressed()) {
                robot.shooter.elevator.toggleTopPos();
            }
            if (gamepad1.leftBumperWasPressed() || gamepad2.leftBumperWasPressed()) {
                robot.shooter.elevator.toggleBottomPos();
            }

            // flower
            if (gamepad1.yWasPressed() || gamepad2.yWasPressed()) {
                robot.toggleFlower();
            }

            robot.update();
            follower.update();

            telemetry.addData("Flower state: ", robot.shooter.isFlowerMode());
            telemetry.addData("Hood pos: ", robot.shooter.hood.getPos());
            telemetry.addData("Alliance: ", HandoffPoseStorage.alliance_color);
            telemetry.addData("Drive mode: ", driveMode);
            telemetry.update();

            TelemetryPacket packet = new TelemetryPacket();
            packet.put("paddle angle", robot.shooter.paddle.getCurrentAngle());
            packet.put("paddle ticks", robot.shooter.paddle.getCurrentTicks());
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
