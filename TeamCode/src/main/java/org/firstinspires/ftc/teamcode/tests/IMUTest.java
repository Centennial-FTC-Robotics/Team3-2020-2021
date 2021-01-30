package org.firstinspires.ftc.teamcode.tests;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.VortechsMethods;

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
