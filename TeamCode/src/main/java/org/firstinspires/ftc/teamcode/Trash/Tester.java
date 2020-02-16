package org.firstinspires.ftc.teamcode.Trash;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.teamcode.Exponential_Methods;
import org.firstinspires.ftc.teamcode.SkystoneDetector;
import org.firstinspires.ftc.teamcode.TensorflowDetector;

import java.util.List;

//@TeleOp(group = "TeleOp", name = "BLUE OPENCV")
public class Tester extends LinearOpMode {
    public void runOpMode() throws InterruptedException {
        SkystoneDetector skystoneDetector = new SkystoneDetector();

        skystoneDetector.initialize(this);
        skystoneDetector.activate("blue");
        waitForStart();


    }

}