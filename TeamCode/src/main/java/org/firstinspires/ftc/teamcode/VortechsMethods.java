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

    public double XPos, YPos, currentAngle, XTarget, YTarget, angleTarget, initialHeading;
    public double tolerance = 0.3;
    public void runOpMode() throws InterruptedException {
        super.runOpMode();
        initializeIMU();
    }

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
        moveAndTurn(sideways, forward,0);
    }
    public void turnRelative(double degrees) {
        moveAndTurn(0,0,degrees - currentAngle);
    }
    public void turn(double degrees) {
        moveAndTurn(0,0,degrees);
    }
    public void moveAndTurn(double xTarget, double yTarget, double angleError) {
        resetEncoders();
        double xError = inchesToTicks(xTarget);
        double prevXError = inchesToTicks(xTarget);
        double yError = inchesToTicks(yTarget);
        double prevYError = inchesToTicks(yTarget);
    //tune the PID here
        double P = 0.005;
        double I = 0.0035;
        double D = 0.00014;

        double angleP = 0.02;
        double angleI = 0.01;
        double angleD = 0.0;

        double prevAngleError = 0.0;
        ElapsedTime timer = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);
        ElapsedTime prevTimer = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);
        while(opModeIsActive() && (yError > tolerance || xError > tolerance || angleError > tolerance)) {

            double yProportion = P * yError;
            double xProportion = P * xError;
            double aProportion = angleP * angleError;

            double yIntegral =+ I * (yError * (timer.milliseconds() - prevTimer.milliseconds()));
            double xIntegral =+ I * (xError * (timer.milliseconds() - prevTimer.milliseconds()));
            double aIntegral =+ angleI * (angleError * (timer.milliseconds() - prevTimer.milliseconds()));
            
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

            frontRight.setPower(frontRightPower + rightTurn);
            backLeft.setPower(frontRightPower + leftTurn);

            frontLeft.setPower(frontLeftPower + leftTurn);
            backRight.setPower(frontLeftPower + rightTurn);

            telemetry.addData("P-value:", P);
            telemetry.addData("I-value:", I);
            telemetry.addData("D-value:", D);
            telemetry.addData("xError", xError);
            telemetry.addData("yError", yError);
            telemetry.addData("frontRightPower", frontRightPower);
            telemetry.addData("frontLeftPower", frontLeftPower);
            telemetry.addData("rightTurn", rightTurn);
            telemetry.addData("leftTurn", leftTurn);
            telemetry.update();
            updateIMU();
        }
    }
    public double ticksToInches(int ticks) {
        return ticks / TICKS_PER_INCH;
    }
    public double inchesToTicks(double inches) {
        return Math.round(inches * TICKS_PER_INCH);
    }

        public void updateIMU() {
    orientation = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
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
        parameters.loggingTag = "imu";
        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();
        imu.initialize(parameters);
    }
    private void resetOrientation(){
        initialHeading = orientation.firstAngle;
    }
    public void setPosition(double XPos, double YPos){
        this.XPos = XPos;
        this.YPos = YPos;
    }



    public void driveStraight(double inches, double power){ resetOrientation();
        int ticks = (int)(TICKS_PER_INCH * inches);

        resetDriveMotors();
        setRunToPosition();

        backLeft.setTargetPosition(ticks);
        backRight.setTargetPosition(ticks);
        frontLeft.setTargetPosition(ticks);
        backRight.setTargetPosition(ticks);

        double drivePower = Range.clip(power,0,1);

        backLeft.setPower(drivePower);
        backRight.setPower(drivePower);
        frontLeft.setPower(drivePower);
        frontRight.setPower(drivePower);

    }

    public void rotate(double degrees){
        double factor = degrees*(TICKS_PER_INCH/180);

        backLeft.setTargetPosition((int) factor);
        backRight.setTargetPosition(-(int) factor);
        frontLeft.setTargetPosition((int) factor);
        frontRight.setTargetPosition(-(int) factor);
    }



}

