package org.firstinspires.ftc.teamcode.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.AutonomousPaths;
import org.firstinspires.ftc.teamcode.SkystoneDetector;

@Autonomous(group = "Autonomous", name = "Red Corner")
public class RedCornerAuto extends AutonomousPaths {
    public void runOpMode() throws InterruptedException { // starts on second tile from the left
        super.runOpMode();
        SkystoneDetector skystoneDetector = new SkystoneDetector();
        skystoneDetector.initialize(this);
        skystoneDetector.activate("red");
        waitForStart();
        int stonePos = skystoneDetector.getStonePos();
        skystoneDetector.deactivate();

        //cornerAuto("red", false, false);

        cornerAutoSideways("red", stonePos);
    }

}