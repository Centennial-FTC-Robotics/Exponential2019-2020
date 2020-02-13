package org.firstinspires.ftc.teamcode.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.Exponential_Methods;
import org.firstinspires.ftc.teamcode.SkystoneDetector;

@Autonomous(group = "Autonomous", name = "Blue Corner")
public class BlueCornerAuto extends Exponential_Methods {
    public void runOpMode() throws InterruptedException { // starts on second tile from the right
        super.runOpMode();
        SkystoneDetector skystoneDetector = new SkystoneDetector();
        skystoneDetector.initialize(this);
        skystoneDetector.activate("blue");
        waitForStart();
        int stonePos = skystoneDetector.getStonePos();
        skystoneDetector.deactivate();
        
        //cornerAuto("red", false, false);

        cornerAutoSideways("blue", stonePos);
    }
}