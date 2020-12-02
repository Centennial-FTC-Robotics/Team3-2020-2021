package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.PIDCoefficients;

class VortechsMethods extends VortechsHardware {

    protected static final double TICKS_PER_INCH = (1120.0 / (100.0 * Math.PI)) * 25.4;
    protected static final double TILE_LENGTH = 24;

    public void launch(double power, long seconds) throws InterruptedException {
        leftOutTake.setPower(power);
        rightOutTake.setPower(-power);
        Thread.sleep(seconds * 1000);
    }

    public void intake(long seconds) throws InterruptedException {
        intakeWheel.setPower(1);
        Thread.sleep(seconds * 1000);
    }

    public void move(double forwards) throws InterruptedException {
        frontRight.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        frontLeft.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        backRight.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        backLeft.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        double p = 0.0;
        double i = 0.0;
        double d = 0.0;

    }

    double YPos, XPos;
    public void update() {
    YPos = frontLeft.getCurrentPosition();
    XPos = frontRight.getCurrentPosition();

    }
    public boolean motorsBusy() {
        return frontRight.isBusy() || frontLeft.isBusy() ||
                backRight.isBusy() || backLeft.isBusy();
    }

}
