package org.firstinspires.ftc.teamcode.Shooter;

//import com.acmerobotics.dashboard.telemetry.TelemetryPacket;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
//import com.acmerobotics.dashboard.telemetry.TelemetryPacket;

public class Launcher extends LinearOpMode {
    DcMotor scooperMotor;

    @Override
    public void runOpMode() throws InterruptedException {
        scooperMotor = hardwareMap.get(DcMotor.class, "launchMotor");
//        scooperMotor.setDirection(DcMotor.Direction.REVERSE);
    }

    public void shootBall() {
        scooperMotor.setPower(1);
    }

    public void flowerBall() {
        scooperMotor.setPower(0.75);
    }

    public void reverseBalls() {
        scooperMotor.setPower(-1);
    }

    public void rest() {
        scooperMotor.setPower(0);
    }

}
