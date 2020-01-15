package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Hardware;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.internal.tfod.Timer;

@TeleOp(name = "FieldCentric", group = "TeleOp")

public class FieldCentric extends TeleOpMethods {
    @Override
    public void driveTrain() {
        double currentAngle = getRotationinDimension('Z') + Math.PI / 2;
        // double currentAngle = 0; //  TODO; FIND OUT HOW TO SET LATeR

        //------FIELD CENTRIC-------
        // THESE ANGLES ARE IN STANDARD POSITION. maybe non standard position works, but only if bearing bearing or std. std.
        double inputX = gamepad1.left_stick_x;
        double inputY = gamepad1.right_stick_y;
        //gets the input angle. Adds pi to tangent if the x value of the angle is less than 0
        double inputAngle = Math.atan(inputY / inputX) + inputX < 0 ? Math.PI : 0;
        double inputMagnitude = Math.sqrt(inputX * inputX + inputY * inputY);

        double angleDistance = inputAngle - currentAngle;
        double angleOnRobot = currentAngle - angleDistance;  //angle of the movement relative to the robot's POV

        double centricX = inputMagnitude * Math.cos(angleOnRobot);
        double centricY = inputMagnitude * Math.sin(angleOnRobot);  //ask yuhwan for the picture if you want, it will not help

        double[] answer = circle_to_taxicab(centricX, centricY, .8*gamepad1.right_stick_x);
        double factor = 1;
        if(gamepad1.left_bumper){
            factor = .5;
        }
        if (gamepad1.right_bumper){
            factor = .25;
        }
        frontRight.setPower(factor*answer[0]);
        backRight.setPower(factor*answer[1]);
        backLeft.setPower(factor*answer[2]);
        frontLeft.setPower(factor*answer[3]);

    }
}