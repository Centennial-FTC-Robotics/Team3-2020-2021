package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.VortechsMethods;

@Autonomous(group = "Autonomous", name = "BackupAuto")
public class BackupAuto extends VortechsMethods {
    // robot parks at launch line and launches rings at high goal
    public void runOpMode() throws InterruptedException {
        waitForStart();
        super.runOpMode();
        //driveStraight(10,0.5);
        //moveRelative(0, 10); //Park on launch line
        driveStraight(20,1);
        conveyorLaunch(2,3); //Shoot rings
    }
}
