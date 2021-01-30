package org.firstinspires.ftc.teamcode.tests;

import org.firstinspires.ftc.teamcode.VortechsMethods;

public class OuttakeTest extends VortechsMethods {
    public void runOpMode() throws InterruptedException {
        super.runOpMode();
        waitForStart();
        while (opModeIsActive()) {
            conveyorLaunch(1, 8);
        }
    }
}
