package org.firstinspires.ftc.teamcode;

public class OuttakeTest extends VortechsMethods {
    public void runOpMode() throws InterruptedException {
        super.runOpMode();
        waitForStart();
        while (opModeIsActive()) {
            conveyorLaunch(1, 8);
        }
    }
}
