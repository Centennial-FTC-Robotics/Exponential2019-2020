package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(group = "Autonomous", name = "BlueCornerAuto")
public class BlueCornerAuto extends Exponential_Methods {
    int factor = -1;

    public void runOpMode() throws InterruptedException {
        super.runOpMode();
        waitForStart();

        telemetry.addData("in run op mode", "");
        telemetry.update();

        move(0, factor * 22.75, 0.5);
        int inches = grabSkystone("red");
        move(0, factor * 22.75 * 5, .5);

    }
}