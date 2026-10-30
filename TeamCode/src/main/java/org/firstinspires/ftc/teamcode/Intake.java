package org.firstinspires.ftc.teamcode;

//import com.acmerobotics.dashboard.telemetry.TelemetryPacket;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
//import com.acmerobotics.dashboard.telemetry.TelemetryPacket;

@Config
public class Intake {
    DcMotor intakeMotor;
    boolean intakeToggle = false;

    public Intake(HardwareMap hardwareMap) {
        intakeMotor = hardwareMap.get(DcMotor.class, "intakeMotor");
//        intakeMotor.setDirection(DcMotor.Direction.REVERSE);
    }

    public void toggle() {
        if (intakeToggle) {
            intakeToggle = false;
        }
        else {
            intakeToggle = true;
        }
    }

    public void runIntake() {
        if (Elevator.elevatorisUp) {
            intakeMotor.setPower(0);
        }
        else if (intakeToggle) {
            intakeMotor.setPower(1);
        }
        else {
            intakeMotor.setPower(0);
        }
    }

    public void spit() {
        intakeMotor.setPower(-1);
    }

    public void rest() {
        intakeMotor.setPower(0);
    }

}
