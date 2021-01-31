package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.VortechsMethods;

@Autonomous(group = "Autonomous", name = "BackupAuto")
public class BackupAuto extends VortechsMethods {
    // robot parks at launch line and launches rings at high goal
    public void runOpMode() throws InterruptedException {
        waitForStart();
        super.runOpMode();
        driveStraight(20,0.5);
        //moveRelative(0, 10); //Park on launch line
        //driveStraightBasic(0.5,2);
        //driveStraightBasic(0.5,2);
        launch(0.65,5); //Shoot rings
        conveyorLaunch(1,3);


        //driveStraight(1,0.5); //park at line
        telemetry.addData("Front left power:", frontLeft.getPower());
        telemetry.addData("Back left power:", backLeft.getPower());
        telemetry.addData("Front right power:", frontRight.getPower());
        telemetry.addData("Back right power:", backRight.getPower());
        telemetry.update();
    }
}
