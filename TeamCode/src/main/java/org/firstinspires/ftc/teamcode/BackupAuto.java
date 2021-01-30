package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.VortechsMethods;

@Autonomous(group = "Autonomous", name = "BackupAuto")
public class BackupAuto extends VortechsMethods {
    // robot parks at launch line and launches rings at high goal
    public void runOpMode() throws InterruptedException {
        super.runOpMode();
        driveStraight(80,0.5);
        //moveRelative(0, 80); //Park on launch line
        conveyorLaunch(2,3); //Shoot rings
    }
}
