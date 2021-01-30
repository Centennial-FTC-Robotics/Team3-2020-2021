package org.firstinspires.ftc.teamcode.tests;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.VortechsMethods;

@Autonomous
public class EncodersTest extends VortechsMethods {
    public void runOpMode() throws InterruptedException {
        super.runOpMode();
        resetDriveMotors();
        setBasicTolerance(4);
        waitForStart();
        while(opModeIsActive()) {

            telemetry.addData("frontLeftPosition:", frontLeft.getCurrentPosition());
            telemetry.addData("frontRightPosition:", frontRight.getCurrentPosition());
            telemetry.addData("backLeftPosition:", backLeft.getCurrentPosition());
            telemetry.addData("backRightPosition:", backRight.getCurrentPosition());
            telemetry.update();
        }
    }
}
