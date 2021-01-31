package org.firstinspires.ftc.teamcode.tests;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.VortechsMethods;
@TeleOp
public class MotorsTest extends VortechsMethods {

    @Override
    public void runOpMode() throws InterruptedException {
        super.runOpMode();
        waitForStart();
        while (opModeIsActive()) {
            if (gamepad1.left_bumper)
                frontLeft.setPower(0.1);
            else
                frontLeft.setPower(0);
            if (gamepad1.right_bumper)
                frontRight.setPower(0.1);
            else
                frontRight.setPower(0);
            if (gamepad1.right_trigger > 0.05)
                backRight.setPower(0.1);
            else
                backRight.setPower(0);
            if (gamepad1.left_trigger > 0.05)
                backLeft.setPower(0.1);
            else
                backRight.setPower(0);

        }
    }
}


