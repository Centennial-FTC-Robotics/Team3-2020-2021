package org.firstinspires.ftc.teamcode;

public class IMUTest extends VortechsMethods {
    @Override
    public void runOpMode() throws InterruptedException {
        super.runOpMode();
        while(opModeIsActive()) {
            updateIMU();
            telemetry.addData("Angle:",currentAngle);
        }
    }
}
