package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.*;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

@Autonomous(group = "Autonomous", name = "Intake Tester")
public class Tester extends LinearOpMode {


    public void runOpMode() throws InterruptedException {

        Servo servoLeft = hardwareMap.servo.get("intakeServoLeft");
        Servo servoRight = hardwareMap.servo.get("intakeServoRight");



        waitForStart();
            servoLeft.setPosition(0);
            sleep(1000);
            telemetry.addData("Position", servoLeft.getPosition());
            telemetry.update();
            servoLeft.setPosition(0);
            sleep(1000);
            servoLeft.setPosition(.4);
            sleep(1000);
            servoLeft.setPosition(.8);


    }
}
