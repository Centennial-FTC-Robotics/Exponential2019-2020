package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(group = "Autonomous", name = "RedCornerAuto")
public class RedCornerAuto extends Exponential_Methods {

    public static final double TILE_LENGTH = 22.75;
    public void runOpMode() throws InterruptedException {
        super.runOpMode();
        waitForStart();
        while(opModeIsActive())
            cornerAuto("red");
        /*while(opModeIsActive())
            run("red");*/
    }




}