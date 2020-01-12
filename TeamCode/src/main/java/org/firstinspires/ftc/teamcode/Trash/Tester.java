package org.firstinspires.ftc.teamcode.Trash;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.*;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.ColorSensor;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Exponential_Methods;

@TeleOp(group = "TeleOp", name = "TESTING STUFF")
public class Tester extends Exponential_Methods {


    public void runOpMode() throws InterruptedException {
        runOpMode();
        boolean isAuto = true;

        waitForStart();
        while(opModeIsActive()){
            if(isAuto){
                move(0,12,1);
                if(gamepad1.x){
                    isAuto = false;
                }
            }
            else{
                if(gamepad1.a)
                    toggleHook(true);
            }

        }


    }
}