package org.firstinspires.ftc.teamcode.Shooter;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

public class Hood extends LinearOpMode {
    Servo hoodServo;

    @Override
    public void runOpMode() throws InterruptedException {
        hoodServo = hardwareMap.get(Servo.class, "hoodServo");
    }

    public void hoodFlower() {
        hoodServo.setPosition(1);
    }

    public void hoodShoot() {
        hoodServo.setPosition(0.5);
    }
}
