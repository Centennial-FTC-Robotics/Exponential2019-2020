package org.firstinspires.ftc.teamcode.Trash;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.Autonomous.AutonomousPaths;
import org.firstinspires.ftc.teamcode.Position;

@Autonomous(group = "Autonomous", name = "moveTo tester")
public class MoveToTester extends AutonomousPaths {
    public void runOpMode() throws InterruptedException {
        super.runOpMode();
        waitForStart();

        currentPosition = new Position(0, 0);

        //top right -> top left -> bottom right -> origin
        moveTo(12, 12);
        moveTo(-12, 12);
        moveTo(12, -12);
        moveTo(0, 0);
        sleep(1500);

        turnRelative(-90);
        moveTo(12, 12);
        moveTo(-12, 12);
        moveTo(12, -12);
        moveTo(0, 0);
        sleep(1500);

        turnRelative(-45);
        moveTo(12, 12);
        moveTo(-12, 12);
        moveTo(12, -12);
        moveTo(0, 0);
        sleep(1500);

        turnRelative(180);
        moveTo(12, 12);
        moveTo(-12, 12);
        moveTo(12, -12);
        moveTo(0, 0);
        /*moveTo(12, 12);
        moveTo(0, 12);
        moveTo(-12, -12);
        moveTo(0, 0);*/

    }
}
