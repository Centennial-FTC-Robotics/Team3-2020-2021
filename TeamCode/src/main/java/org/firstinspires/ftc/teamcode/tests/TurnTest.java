package org.firstinspires.ftc.teamcode.tests;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.VortechsMethods;

@Autonomous(group = "Autonomous", name = "TurnTest")
public class TurnTest extends VortechsMethods {
    public void runOpMode() throws InterruptedException {
        super.runOpMode();
        resetDriveMotors();

        waitForStart();

        turnRelative(120);
/*        sleep(500);
        move(4,0);*/
        //move(-10,0);
    }
}
