package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.PIDCoefficients;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.Position;

class VortechsMethods extends VortechsHardware {

    protected static final double TICKS_PER_INCH = (1120.0 / (100.0 * Math.PI)) * 25.4;
    protected static final double TILE_LENGTH = 24;
    private final DcMotorEx[] driveMotors = {frontLeft, frontRight, backLeft, backRight};


    public double XPos, YPos, currentAngle, XTarget, YTarget, angleTarget, initialHeading;

    public void launch(double power, long seconds) throws InterruptedException {
        leftOutTake.setPower(power);
        rightOutTake.setPower(-power);
        Thread.sleep(seconds * 1000);
    }

    public void intake(long seconds) throws InterruptedException {
        intakeWheel.setPower(1);
        Thread.sleep(seconds * 1000);
    }

    public void moveRelative(double sideways, double forward){
        moveAndTurn(sideways - XPos, forward - YPos,0);
    }
    public void turnRelative(double degrees) {
        moveAndTurn(0,0,degrees - currentAngle);
    }
    public void turn(double degrees) {
        moveAndTurn(0,0,degrees);
    }
    public void moveAndTurn(double XTarget, double YTarget, double angleTarget) {
        for(DcMotorEx motor : driveMotors) {
            motor.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
            motor.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        }//tune the PID here
        double P = 0.5;
        double I = 0.0;
        double D = 0.0;

        double angleP = 0.5;
        double angleI = 0.0;
        double angleD = 0.0;

        double yError = 0.0;
        double xError = 0.0;

        double angleError;    //the distance from the target
        double prevYError = 0.0, prevXError = 0.0, prevAngleError = 0.0;
        ElapsedTime timer = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);
        ElapsedTime prevTimer = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);
        while(opModeIsActive()) {

            angleError = angleTarget - currentAngle;

            double yProportion = P * yError;
            double xProportion = P * xError;
            double aIntegral = angleP * angleError;

            double yIntegral =+ I * (yError * (timer.milliseconds() - prevTimer.milliseconds()));
            double xIntegral =+ I * (xError * (timer.milliseconds() - prevTimer.milliseconds()));
            double aProportion =+ angleI * (angleError * (timer.milliseconds() - prevTimer.milliseconds()));
            
            double yDerivative = D * (yError - prevYError) / (timer.milliseconds() - prevTimer.milliseconds());
            double xDerivative = D * (xError - prevXError) / (timer.milliseconds() - prevTimer.milliseconds());
            double aDerivative = angleD * (angleError - prevAngleError) / (timer.milliseconds() - prevTimer.milliseconds());
            
            prevTimer = timer;
            prevYError = yError;
            prevXError = xError;
            prevAngleError = angleError;

            double frontRightPower = (P * yProportion + I * yIntegral - D * yDerivative) + 
                    (P * xProportion + I * xIntegral + D * xDerivative);
            double frontLeftPower = (P * yProportion + I * yIntegral - D * yDerivative) -
                    (P * xProportion + I * xIntegral + D * xDerivative);

            double rightTurn = angleP * aProportion + angleI * aIntegral - angleD * aDerivative;
            double leftTurn = -angleP * aProportion - angleI * aIntegral + angleD * aDerivative;

            frontRight.setPower(Range.clip(frontRightPower + rightTurn, 0, 1));
            backLeft.setPower(Range.clip(frontRightPower + leftTurn,0,1));

            frontLeft.setPower(Range.clip(frontLeftPower + leftTurn,0,1));
            backRight.setPower(Range.clip(frontLeftPower + rightTurn,0,1));
        }
    }
    public double ticksToInches(int ticks) {
        return ticks/TICKS_PER_INCH;
    }
    public void updateIMU() {
    orientation = IMU.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
    currentAngle = AngleUnit.normalizeDegrees(orientation.firstAngle - initialHeading);
    }
    public boolean motorsBusy() {
        return frontRight.isBusy() || frontLeft.isBusy() ||
                backRight.isBusy() || backLeft.isBusy();
    }
    Orientation orientation = new Orientation();
    public void initializeIMU() {
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "AdafruitIMUCalibration.json";
        parameters.loggingEnabled = true;
        parameters.loggingTag = "IMU";
        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();
        IMU.initialize(parameters);
    }
    private void resetOrientation(){
        initialHeading = orientation.firstAngle;
    }
    public void setPosition(double XPos, double YPos){
        this.XPos = XPos;
        this.YPos = YPos;
    }



}

