package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

@Config
public class Flower {
    Servo flowerServo;
    Servo descoreServo;
    boolean toggleFlowerDescorer = true;

    public Flower(HardwareMap hardwareMap) {
        flowerServo = hardwareMap.get(Servo.class, "flowerServo");
        descoreServo = hardwareMap.get(Servo.class, "descoreServo");
        descoreServo.setPosition(0.55);
    }

    public void extendFlowerServo() {
        flowerServo.setPosition(0.5);
    }

    public void toggleFlowerDescorer() {
        if (toggleFlowerDescorer) {
            toggleFlowerDescorer = false;
            descoreServo.setPosition(0.55);
        }
        else {toggleFlowerDescorer = true;
            descoreServo.setPosition(0);
        }
    }
}
