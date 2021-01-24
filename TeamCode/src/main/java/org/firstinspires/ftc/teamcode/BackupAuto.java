package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.VortechsMethods;

@Autonomous(group = "Autonomous", name = "BackupAuto")
public class BackupAuto extends VortechsMethods {
    // robot parks at launch line and
    public void runOpMode() throws InterruptedException {
        super.runOpMode();
        moveRelative(0, 80); //Park on white line
    }


}
