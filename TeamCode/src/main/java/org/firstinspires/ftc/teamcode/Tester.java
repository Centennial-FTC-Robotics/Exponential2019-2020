package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.*;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.ColorSensor;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

import org.firstinspires.ftc.robotcore.external.Telemetry;

@Autonomous(group = "Autonomous", name = "Intake Tester")
public class Tester extends Exponential_Methods {


    public void runOpMode() throws InterruptedException {
        super.runOpMode();
        initializeIMU();

        waitForStart();
        while(opModeIsActive()){
            turnRelative(90);
            sleep(1000);
            move(12,0,0.2);
            sleep(1000);
            move(0,12,0.2);
            sleep(  1000);
            move(-12,0,0.2);
            sleep(1000);
            move(0,-12,0.2);
            sleep(1000);
            move(12,12,0);
            sleep(1000);
            move(-12,-12,0);
            sleep(1000);
            turnAbsolute(0);

            telemetry.update();

        }


    }
}
