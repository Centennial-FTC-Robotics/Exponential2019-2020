package org.firstinspires.ftc.teamcode.Trash;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.teamcode.Exponential_Methods;
import org.firstinspires.ftc.teamcode.SkystoneDetector;
import org.firstinspires.ftc.teamcode.TensorflowDetector;

import java.util.List;

@TeleOp(group = "TeleOp", name = "margo testing")
public class Tester extends Exponential_Methods {
    public void runOpMode() throws InterruptedException {
        super.runOpMode();
        telemetry.addData("init", initialHeading);
        initialHeading -= 270;
        telemetry.update();

        waitForStart();
        while(opModeIsActive()){
            telemetry.addData("init", initialHeading);
            telemetry.addData("norm", getRotationInDimension('Z'));
            telemetry.addData("firstAngle", orientation.firstAngle);
            telemetry.addData("rot", getAngle());
            telemetry.update();
        }


    }

}