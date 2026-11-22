package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp @Config
public class hoodTestOP extends LinearOpMode {
    Servo hoodServo;

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
        hoodServo = hardwareMap.get(Servo.class, "hoodServo");

        waitForStart();
        if (isStopRequested()) return;

        while (opModeIsActive()) {
            hoodServo.setPosition(0.8);
        }

    }
}
