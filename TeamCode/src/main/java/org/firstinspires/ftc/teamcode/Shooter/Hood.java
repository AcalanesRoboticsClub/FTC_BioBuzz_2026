package org.firstinspires.ftc.teamcode.Shooter;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

@Config
public class Hood {
    Servo hoodServo;

    public Hood(HardwareMap hardwareMap) {
        hoodServo = hardwareMap.get(Servo.class, "hoodServo");
    }

    public void hoodFlower() {
        hoodServo.setPosition(0.8);
    }

    public void hoodShoot() {
        hoodServo.setPosition(-0.2);
    }
}
