package org.firstinspires.ftc.teamcode;


import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name = "ServoTest", group = "TeleOp")
public class ServoTest extends LinearOpMode {


    public void runOpMode() throws InterruptedException{

        Servo intakeServoRight = hardwareMap.servo.get("intakeServoRight");
        Servo intakeServoLeft = hardwareMap.servo.get("intakeServoLeft");
        waitForStart();
        while(opModeIsActive()){
            if(gamepad2.b){
                intakeServoLeft.setPosition(0);
                intakeServoRight.setPosition(0);
            } else if(gamepad2.x){

                intakeServoLeft.setPosition(1);
                intakeServoRight.setPosition(1);
            }
        }







    }
}
