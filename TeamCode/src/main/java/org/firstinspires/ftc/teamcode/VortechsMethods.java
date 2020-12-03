package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.PIDCoefficients;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;
import org.firstinspires.ftc.robotcore.external.navigation.Position;

class VortechsMethods extends VortechsHardware {

    protected static final double TICKS_PER_INCH = (1120.0 / (100.0 * Math.PI)) * 25.4;
    protected static final double TILE_LENGTH = 24;
    private final DcMotorEx[] driveMotors = {frontLeft, frontRight, backLeft, backRight};

    public void launch(double power, long seconds) throws InterruptedException {
        leftOutTake.setPower(power);
        rightOutTake.setPower(-power);
        Thread.sleep(seconds * 1000);
    }

    public void intake(long seconds) throws InterruptedException {
        intakeWheel.setPower(1);
        Thread.sleep(seconds * 1000);
    }

    public void moveTo(double XTarget, double YTarget) {
        for(DcMotorEx motor : driveMotors) {
            motor.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
            motor.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        }
        double P = 0.5, I = 0.0, D = 0.0;    //tune the PID here
        double tolerance = 1.0;
        double yError, xError;    //the distance from the target
        double prevYError = 0.0, prevXError = 0.0;
        ElapsedTime timer = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);
        ElapsedTime prevTimer = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);
        while(opModeIsActive()) {
            yError = YTarget - YPos;
            xError = XTarget - XPos;

            double yProportion = P * yError;
            double xProportion = P * xError;

            double yIntegral =+ I * (yError * (timer.milliseconds() - prevTimer.milliseconds()));
            double xIntegral =+ I * (xError * (timer.milliseconds() - prevTimer.milliseconds()));
            
            double yDerivative = D * (yError - prevYError) / (timer.milliseconds() - prevTimer.milliseconds());
            double xDerivative = D * (xError - prevXError) / (timer.milliseconds() - prevTimer.milliseconds());
            
            prevTimer = timer;
            double frontRightPower = (P * yProportion + I * yIntegral - D * yDerivative) + 
                    (P * xProportion + I * xIntegral + D * xDerivative);
            double frontLeftPower = (P * yProportion + I * yIntegral - D * yDerivative) +
                    (P * xProportion + I * xIntegral + D * xDerivative);
                    
            frontRight.setPower(frontRightPower);
            backLeft.setPower(frontRightPower);

            frontLeft.setPower(frontLeftPower);
            backRight.setPower(frontLeftPower);
        }
    }

    public double ticksToInches(int ticks) {
        return ticks/TICKS_PER_INCH;
    }

    public double XPos, YPos, XTarget, YTarget;

    public void update() {
    YPos = ticksToInches(frontLeft.getCurrentPosition());
    XPos = ticksToInches(frontRight.getCurrentPosition());
    }

    public boolean motorsBusy() {
        return frontRight.isBusy() || frontLeft.isBusy() ||
                backRight.isBusy() || backLeft.isBusy();
    }
}
