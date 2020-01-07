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


        waitForStart();
        while(opModeIsActive()){

            int pos = 0;
            telemetry.addData("pos", pos);
            telemetry.update();

            pos += gamepad1.left_stick_x * .1;

            if(gamepad1.a){
                hookServoRight.setPosition(pos);
                hookServoLeft.setPosition(pos);
            }

            /*
            if(gamepad1.a)
                toggleHook(true);
            if(gamepad1.b)
                toggleHook(false);

            if(gamepad1.x)
                setIntakeWheels(.5);

            if(gamepad1.y)
                setIntakeWheels(-.5);
            */

            /*
            //Slides
            telemetry.addData("i am here", 0);
            setSlidePower(0.2);
            sleep(2000);
            setSlidePower(-0.2);
            sleep(2000);
            setSlidePower(0);

            telemetry.addData("let there be light", 0);
            extendSlidesTo(4,0.2);

             */
            /*
            //Encoders
            telemetry.addData("front left", frontLeft.getCurrentPosition());
            telemetry.addData("front right", frontRight.getCurrentPosition());
            telemetry.addData("back right", backRight.getCurrentPosition());
            telemetry.addData("back left", backLeft.getCurrentPosition());
            telemetry.update();
            */

            /*
            //Intake
            intakeServoLeft.setPosition(0);
            intakeServoRight.setPosition(0);
            */

            /*
            //Drivetrain
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
            */

            /*
            //Imu
            telemetry.addData("Z", getRotationinDimension('Z'));
            telemetry.addData("X", getRotationinDimension('X'));
            telemetry.addData("Y", getRotationinDimension('Y'));
            telemetry.update();
             */
        }


    }
}