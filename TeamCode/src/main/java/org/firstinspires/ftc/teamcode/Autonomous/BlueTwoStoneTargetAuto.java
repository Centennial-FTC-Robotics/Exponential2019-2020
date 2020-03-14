package org.firstinspires.ftc.teamcode.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.SkystoneDetector;

@Autonomous(group = "Autonomous", name = "Blue 2 Stone Target")
public class BlueTwoStoneTargetAuto extends AutonomousPaths {
    public void runOpMode() throws InterruptedException {
        super.runOpMode();
        SkystoneDetector skystoneDetector = new SkystoneDetector();
        skystoneDetector.initialize(this);
        skystoneDetector.activate("red");
        waitForStart();
        int stonePos = 2 - skystoneDetector.getStonePos();
        skystoneDetector.deactivate();

        twoStoneAutoTargetPosition("blue", stonePos);
    }
}
