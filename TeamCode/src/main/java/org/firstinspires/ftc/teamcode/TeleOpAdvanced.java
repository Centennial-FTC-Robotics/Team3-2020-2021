package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotorEx;

public class TeleOpAdvanced extends VortechsMethods {
    //This code is based on the "Controls" spreadsheet we made in the Google Drive


    double y, x, rotation, speed;

    public void runOpMode() {

        waitForStart();

        while (opModeIsActive()) {
            y = gamepad1.left_stick_y;
            x = -gamepad1.left_stick_x; //flipped


            //slow rotation using triggers
            if (gamepad1.left_trigger > 0.0) {
                rotation = -gamepad1.left_trigger;
            } else if (gamepad1.right_trigger > 0.0) {
                rotation = gamepad1.right_trigger;
            }
            if (gamepad1.dpad_left) {
                turnRelative(-90);
            } else if (gamepad1.dpad_right) {
                turnRelative(90);
            }

            //the x button slows down speed
            if (gamepad1.x) {
                speed = 0.5;
            } else {
                speed = 1.0;
            }

            frontLeft.setPower((y + x + rotation) * speed);
            frontRight.setPower((y - x - rotation) * speed);
            backLeft.setPower((y - x + rotation) * speed);
            backRight.setPower((y + x - rotation) * speed);

            idle();

        }
    }
}
