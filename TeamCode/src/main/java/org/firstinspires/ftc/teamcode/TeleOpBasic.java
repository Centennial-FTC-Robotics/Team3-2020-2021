package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;

public class TeleOpBasic extends VortechsHardware{

    double MotorPower;
    double Steering;
    double RightPower;
    double LeftPower;

    public void runOpMode(){

        frontLeft.setDirection(DcMotor.Direction.REVERSE);
        backLeft.setDirection((DcMotor.Direction.REVERSE));

        waitForStart();

        while(opModeIsActive())
        {
            MotorPower = gamepad1.left_stick_y;
            Steering = gamepad1.left_stick_x;
            RightPower = MotorPower - Steering;
            LeftPower = MotorPower + Steering;

            frontLeft.setPower(LeftPower);
            backLeft.setPower(LeftPower);
            frontRight.setPower(RightPower);
            backRight.setPower((RightPower));

            idle();
        }
    }
}
