package org.firstinspires.ftc.teamcode.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name = "FieldCentric", group = "TeleOp")

public class FieldCentric extends TeleOpMethods {
    @Override
    public void driveTrain() {
        double currentAngle = Math.PI*getRotationinDimension('Z')/180;
        // double currentAngle = 0; //  TODO; FIND OUT HOW TO SET LATeR

        //------FIELD CENTRIC-------
        // THESE ANGLES ARE IN STANDARD POSITION. maybe non standard position works, but only if bearing bearing or std. std.
        double inputX = gamepad1.left_stick_x;
        double inputY = -1*gamepad1.left_stick_y;
        //gets the input angle. Adds pi to tangent if the x value of the angle is less than 0
        // double inputAngle = Math.atan(inputY / inputX) + inputX < 0 ? Math.PI : 0;
        // double inputMagnitude = Math.sqrt(inputX * inputX + inputY * inputY);

        //double angleDistance = inputAngle - currentAngle;
        //double angleOnRobot = currentAngle - angleDistance;  //angle of the movement relative to the robot's POV






        double centricX = inputX*Math.cos(currentAngle)+inputY*Math.sin(currentAngle);
        double centricY = inputX*Math.sin(currentAngle)-inputY*Math.cos(currentAngle);
        telemetry.addData("inputX", inputX);
        telemetry.addData("inputY", inputY);

        telemetry.addData("centricX", centricX);
        telemetry.addData("centricY", centricY);
        telemetry.update();

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