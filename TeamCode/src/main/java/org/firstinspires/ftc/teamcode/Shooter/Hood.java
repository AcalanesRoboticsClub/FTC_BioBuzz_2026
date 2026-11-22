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
    double position = 0.5; // position of hood

    public Hood(HardwareMap hardwareMap) {
        hoodServo = hardwareMap.get(Servo.class, "hoodServo");
    }

    /**
     * Gets position of servo
     * @return position of servo
     */
    public double getPos() {
        return hoodServo.getPosition();
    }

    /**
     * Sets position of servo
     * @param pos postiion to set servo to
     */
    public void setPos(double pos) { position = pos; }

    /*
     * Presets for hood positions
     */
    public void hoodFlower() { position = 0.8; }
    public void hoodShootLow() { position = 0; }
    public void hoodShootHigh() { position = 0.8; }
    public void hoodShootMidHigh() { position = 0.3; }
    public void hoodShootMidLow() { position = 0.5; }

    /*
     * Increment/decrement hood position
     */
    public void incPosition_small() { position += 0.01; }
    public void decPosition_small() { position -= 0.01; }
    public void incPosition() { position += 0.1; }
    public void decPosition() { position -= 0.1; }

    /**
     * Updates hood position
     */
    public void updateHood() {
        hoodServo.setPosition(position);

        TelemetryPacket packet = new TelemetryPacket();

        packet.put("Hood pos", position);

        FtcDashboard.getInstance().sendTelemetryPacket(packet);
    }
}
