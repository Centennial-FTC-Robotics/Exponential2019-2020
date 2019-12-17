package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(group = "Autonomous", name = "BlueCornerAuto")
public class BlueCornerAuto extends RedCornerAuto {
    int factor = -1;

    public void runOpMode() throws InterruptedException {
        super.runOpMode();
        waitForStart();

        grabSkystone("blue");

    }
}