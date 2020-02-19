package org.firstinspires.ftc.teamcode.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.Exponential_Methods;

@Autonomous(group = "Autonomous", name = "Tile Right")
public class TileRight extends AutonomousPaths {
    public void runOpMode() throws InterruptedException {
        super.runOpMode();
        waitForStart();
        tileSideways("right");
    }
}
