package org.firstinspires.ftc.teamcode.Shooter;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

@Config
public class Hood {
    Servo hoodServo;
    double position = 0.4;

    public Hood(HardwareMap hardwareMap) {
        hoodServo = hardwareMap.get(Servo.class, "hoodServo");
    }

    public void hoodFlower() {
        hoodServo.setPosition(0.8);
    }

    public void hoodShootLow() {
        hoodServo.setPosition(0);
    }

    public void hoodShootHigh() {
        hoodServo.setPosition(0.7);
    }

    public void incPosition() {
        position += 0.1;
    }

    public void decPosition() {
        position -= 0.1;
    }

    public void updateHood() {
        hoodServo.setPosition(position);

        TelemetryPacket packet = new TelemetryPacket();

        packet.put("Hood pos", position);

        FtcDashboard.getInstance().sendTelemetryPacket(packet);
    }
}
