package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@Autonomous(name= "Failed_Encoder_Stuff",  group="Autonomous")
public class Failed_Encoder_Stuff extends LinearOpMode {
    DcMotor upleft;
    DcMotor backleft;
    DcMotor backright;
    DcMotor upright;

    public void move(double horizontal, double vertical){

    }
    public void runOpMode() throws InterruptedException {
        upleft = hardwareMap.dcMotor.get("rmotor1");
        backleft = hardwareMap.dcMotor.get("rmotor0");
        backright = hardwareMap.dcMotor.get("lmotor0");
        upright = hardwareMap.dcMotor.get("lmotor1");

        upleft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backleft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        upright.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backright.setMode(DcMotor.RunMode.RUN_USING_ENCODER);



        while(opModeIsActive()){

        }


    }
}
