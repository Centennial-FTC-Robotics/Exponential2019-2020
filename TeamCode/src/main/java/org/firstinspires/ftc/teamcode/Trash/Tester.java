package org.firstinspires.ftc.teamcode.Trash;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.teamcode.Exponential_Methods;
import org.firstinspires.ftc.teamcode.TensorflowDetector;

import java.util.List;

@Autonomous(group = "Autonomous", name = "Side camera vuforia stuff")
public class Tester extends LinearOpMode {
    public void runOpMode() throws InterruptedException {
        TensorflowDetector skystoneDetector = new TensorflowDetector();
        skystoneDetector.initialize(this);
        int stonePos = skystoneDetector.findStone();
        telemetry.addData("stonePos", stonePos);
        telemetry.update();
        waitForStart();
    }

}