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
//    boolean intakeToggle = false;
    boolean overrideIntake = false;
    enum STATE {REST, INTAKE, FORCE_INTAKE, SPIT};
    STATE intakeToggle = STATE.REST;

    public Intake(HardwareMap hardwareMap) {
        intakeMotor = hardwareMap.get(DcMotor.class, "intakeMotor");
//        intakeMotor.setDirection(DcMotor.Direction.REVERSE);
    }

    public void toggle() {
        if (intakeToggle == STATE.REST) {
            intakeToggle = STATE.INTAKE;
        }
        else if (Elevator.elevatorisUp && (intakeToggle == STATE.REST || intakeToggle == STATE.INTAKE)) {
            intakeToggle = STATE.FORCE_INTAKE;
        }
        else {
            intakeToggle = STATE.REST;
        }
    }

    public void runIntake() {
        if (intakeToggle == STATE.SPIT) {
            intakeMotor.setPower(-1);
            intakeToggle = STATE.REST;
        }
        else if (Elevator.elevatorisUp && intakeToggle != STATE.FORCE_INTAKE) {
            intakeMotor.setPower(0);
        }
        else if (intakeToggle == STATE.INTAKE || intakeToggle == STATE.FORCE_INTAKE) {
            intakeMotor.setPower(1);
        }
        else {
            intakeMotor.setPower(0);
        }
    }

    public void overrideIntake() {
        if (overrideIntake) {
            overrideIntake = false;
        }
        else {
            overrideIntake = true;
        }
    }

    public void spit() {
        intakeToggle = STATE.SPIT;
    }

    public void rest() {
        intakeToggle = STATE.REST;
    }

}
