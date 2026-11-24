package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.pedropathing.math.Pose;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Shooter.Shooter;

/*
 * Robot class
 *   Holds all mechanisms for the robot (driving is handled by Pedro)
 *   shooter - paddle, hood, and elevator
 *   flower - flower scoring rail and descorer
 *   intake - intake motors
 */

@Config
public class Robot {
    // Mechanisms
    Intake intake;
    Shooter shooter;
    Flower flower;
    Servo ledIndicator;

    double timeForPaddleStart = 100;
    boolean runPaddle = false;

    // Robot pose
    public static Pose robotPose;
    VoltageSensor vs;
    double volts;

    ElapsedTime timer = new ElapsedTime();
    ElapsedTime voltTimer = new ElapsedTime();
    ElapsedTime patternTimer = new ElapsedTime();

    public Robot(HardwareMap hardwareMap) {
        shooter = new Shooter(hardwareMap);
        intake = new Intake(hardwareMap);
        flower = new Flower(hardwareMap);

        ledIndicator = hardwareMap.get(Servo.class, "led");

        vs = hardwareMap.voltageSensor.iterator().next(); // get battery voltage

        timer.reset();
        voltTimer.reset();
    }

    /**
     * Runs at the start of the match to deploy all mechanisms
     */
    public void deploy() {
        shooter.deploy();
        flower.extendFlowerServo();
        flower.toggleFlowerDescorer();
    }

    /**
     * Toggles flower mode on and off
     */
    public void toggleFlower() {
        shooter.toggleFlowerMode();
        if (!shooter.isFlowerMode()) {
            shooter.setShootProfile();
        }
    }

    /**
     * Spins paddle and intake in the reverse direction
     */
    public void spitAll() {
        shooter.paddle.spit();
        intake.spit();
    }

    /**
     * Updates and runs all robot mechanisms
     * Paddle mechanism has an initial delay to start
     */
    public void update() {
        // Only updates voltage every 0.5 seconds
        if (voltTimer.seconds() > 0.5) {
            volts = vs.getVoltage();
            voltTimer.reset();
        }

        // Blinking lights
        // green - flower
        // purple - shoot
        // number yellow blinks - pollen
        // number blue blinks - nectar
        // number red blinks - nectar
        double backgroundColor = 0; // off - shoot
        if (shooter.isFlowerMode()) {
            backgroundColor = 0.0; // green - flower
        }
        double cycleTime = patternTimer.seconds();
        double timeToBlink = 0.25;
        double timeForPollen = shooter.numPollen * timeToBlink;
        double timeForNectar = shooter.numNectar * timeToBlink;
        if (cycleTime < 1) {
            ledIndicator.setPosition(backgroundColor);
        }
        else if (cycleTime > 1 && cycleTime < 1 + timeForPollen && timeForPollen > 0.1) {
            double timeInStage = cycleTime - 0.5;
            double timeInCurrentStep = timeInStage % timeToBlink;

            if (timeInCurrentStep < timeToBlink / 2.0) {
                ledIndicator.setPosition(0.333);
            } else {
                ledIndicator.setPosition(backgroundColor);
            }
        }
        else if (cycleTime > 1.5 + timeForPollen && cycleTime < 1.5 + timeForPollen + timeForNectar && timeForNectar > 0.1) {
            double timeInStage = cycleTime - 0.5 - timeForPollen;
            double timeInCurrentStep = timeInStage % timeToBlink;

            if (timeInCurrentStep < timeToBlink / 2.0) {
                if (HandoffPoseStorage.alliance_color == HandoffPoseStorage.ALLIANCE.BLUE) {
                    ledIndicator.setPosition(0.611);
                }
                if (HandoffPoseStorage.alliance_color == HandoffPoseStorage.ALLIANCE.RED) {
                    ledIndicator.setPosition(0.277);
                }
            } else {
                ledIndicator.setPosition(backgroundColor);
            }
        }
//        else if (cycleTime < 2 + timeForPollen + timeForNectar) {
//            ledIndicator.setPosition(0.0);
//        }
        else {
            patternTimer.reset();
        }


        double dt = timer.seconds();
        timer.reset();

        // Delay for paddle to start to give time to flower servo
        if (timeForPaddleStart <= 0 && !runPaddle) {
            shooter.paddle.deploy();
            runPaddle = true;
        }
        else {
            timeForPaddleStart -= 2;
        }

        // Updates intake and shooter
        intake.updateIntake();
        shooter.updateShooter(dt, volts, runPaddle);

        // Telemetry
        TelemetryPacket packet = new TelemetryPacket();

//        packet.put("Paddle LAUNCH_STATE", Shooter.paddle.Paddle_state.toString());
//        packet.put("Paddle power", shoot_power);
//        packet.put("Paddle shoot toggle", PaddleTypeToggle.toString());
//        packet.put("Current Angle", currentAngle);
//        packet.put("Target Angle", targetAngle);
//        packet.put("Error", error);
//        packet.put("PID Output", pidOutput);

//        packet.put("kP", Paddle.PID_COEFFS.kP);
//        packet.put("kI", Paddle.PID_COEFFS.kI);
//        packet.put("kD", Paddle.PID_COEFFS.kD);
        packet.put("FLOWER MODE", shooter.isFlowerMode());

        FtcDashboard.getInstance().sendTelemetryPacket(packet);
    }
}
