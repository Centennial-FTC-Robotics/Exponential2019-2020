package org.firstinspires.ftc.teamcode.Trash;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.teamcode.Exponential_Methods;
import org.firstinspires.ftc.teamcode.SkystoneDetector;

import java.util.List;

@TeleOp(group = "TeleOp", name = "OpenCV Tester")
public class OpenCVTester extends LinearOpMode {
    public SkystoneDetector skystoneDetector = new SkystoneDetector();

    public void runOpMode() throws InterruptedException {
        skystoneDetector.initialize(this);
        skystoneDetector.activate("red");
        waitForStart();


    }



}