package org.firstinspires.ftc.teamcode.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.AutonomousPaths;
import org.firstinspires.ftc.teamcode.SkystoneDetector;

@Autonomous(group = "Autonomous", name = "Red 2 Stone")
public class RedTwoStoneAuto extends AutonomousPaths {
    public void runOpMode() throws InterruptedException {
        super.runOpMode();
        SkystoneDetector skystoneDetector = new SkystoneDetector();
        skystoneDetector.initialize(this);
        skystoneDetector.activate("blue");
        waitForStart();
        int stonePos = 2 - skystoneDetector.getStonePos();
        skystoneDetector.deactivate();

        twoStoneAuto("red", stonePos);
    }
}
