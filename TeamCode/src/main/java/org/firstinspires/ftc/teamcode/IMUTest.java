package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous
public class IMUTest extends VortechsMethods {
    @Override
    public void runOpMode() throws InterruptedException {
        super.runOpMode();
        waitForStart();
        while(opModeIsActive()) {
            updateIMU();
            telemetry.addData("Angle:",currentAngle);
            telemetry.update();
        }
    }
}
