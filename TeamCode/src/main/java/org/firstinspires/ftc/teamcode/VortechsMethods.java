package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.PIDCoefficients;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.State;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;
import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer.CameraDirection;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;

import java.util.List;

public class VortechsMethods extends VortechsHardware {

    public boolean toggle = false;
    public boolean toggle2 = false;

    private final PIDController diag1Controller = new PIDController(0.100f, 0, 0);
    private final PIDController diag2Controller = new PIDController(0.100f, 0, 0);

    private final double ticksPerRotation = 537.6 * 2; //IMPORTANT: for some reason the robot moved half of its distance so i added a *2
    private final double diameter = 3.937;
    private final double ticksPerWheelRotation = ticksPerRotation;
    private final double Circumference = diameter * Math.PI;
    private final double ticksPerInch = Circumference / ticksPerWheelRotation;

    private TFObjectDetector tfod;
    private VuforiaLocalizer vuforia;

    public double XPos, YPos, currentAngle, XTarget, YTarget, angleTarget, initialHeading;
    public double tolerance = 0.3;

    protected static final double TARGET_A = 0;
    protected static final double TARGET_B = 1;
    protected static final double TARGET_C = 2;

    private static final String TFOD_MODEL_ASSET = "UltimateGoal.tflite";
    private static final String LABEL_FIRST_ELEMENT = "Quad";
    private static final String LABEL_SECOND_ELEMENT = "Single";

    private static final String VUFORIA_KEY =
            "AaE18sD/////AAABmZ7zTjrwDEwgoUUd9Hg/fVNlCi1mnUJCizFmysoKuVPPNnIEWJmK9SlpRppNs0SV9sDdCFc6nySaX1KM3CimlwDwzEcmZs016lHBxh3A0S5hVFPPHWzE34TCYgA90g9nrKrwRIFolSSO6p9YmDLzi4fFHcOe85nuiYRfFZwaYlCnTZnwU3czaUue9uFiq3Q9e9Hytr3EtxJrvKISSdNah+WP+43QaqrLcQR7NfOkYQ5AY+omdtZ76KfgooK5dtO4lgYwxAWkVVAYt60zLcrpd4ZHC9Nu+6xkhLF5QhJDbfSUD0/Kep5MhZqugCpguNDzvcBQ5HtCVYvjGYO6pe7Gy6JwRiB1E2gAatjyS0Prc2pV";

    // hardcode constants for distances/lengths on the field here
    // this will make it easier to mirror the red and blue auto paths

    protected static final double TILE_LENGTH = 24;


    @Override
    public void runOpMode() throws InterruptedException {
        super.runOpMode();
        initializeIMU();
    }

    // Mechanisms

    public void Toggle() {
        toggle = !toggle;
    }

    public void Toggle2() {
        toggle2 = !toggle2;
    }

    public void controlWobbleArm() {
        grabberArm.setPosition(0.1);
        grabberHand.setPosition(0.8);
        sleep(1000);
        grabberHand.setPosition(0.25);
        sleep(1000);
        grabberArm.setPosition(0.6);
        sleep(500);
    }
    public void autoControlWobbleArm(){
//        grabberHand.setPosition(0.4);
        grabberArm.setPosition(0.1);
        sleep(1000);
        grabberHand.setPosition(0.8);
        sleep(1000);
        grabberHand.setPosition(0.4);
        sleep(1000);
        grabberArm.setPosition(0.6);
        sleep(500);
    }

    public void initWobbleArm(){
        grabberArm.setPosition(.8);
        grabberHand.setPosition(0);
        sleep(1000);
    }

 /*   public void sleep(double milliseconds) {
        ElapsedTime time = new ElapsedTime();
        while (opModeIsActive()) { sleep();
            if (time.milliseconds() < milliseconds) {
                idle();
            } else {
                break;
            }
        *//*while (opModeIsActive() && time.milliseconds() < milliseconds) {
        it kept yelling at me for being an idiot that uses while loops
        }*//*
        }
    }*/

    public void launch(double power, long seconds) throws InterruptedException {
        leftOutTake.setPower(-power);
        //rightOutTake.setPower(-power);
        sleep(1000);
    }

    public void conveyorLaunch(double power, long seconds) throws InterruptedException {
        launch(power, seconds);
        conveyor(1, seconds);
    }

    public void conveyor(double power, long seconds) throws InterruptedException {
        conveyor.setPower(-power);
        sleep(1000);
    }

    public void intake(long seconds) throws InterruptedException {
        intakeWheel.setPower(1);
        sleep(1000);
    }

    public void outakeWithEncoders(double power){
        resetOutTake();
        leftOutTake.setTargetPosition(5000); //change number of ticks accordingly
        leftOutTake.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        leftOutTake.setPower(-power);

        telemetry.addData("current position:", leftOutTake.getTargetPosition());
        telemetry.update();
    }


    public void doEverything(double outtakePower, double conveyorPower, long seconds) throws InterruptedException {

        //launch the first two rings
        conveyor.setPower(-conveyorPower);
        leftOutTake.setPower(-outtakePower);

       /* The sleep methods don't play nice with the velocity pid thing, idk how to fix
       resetOutTake();
        leftOutTake.setVelocity(1000.0);
        telemetry.addData("velocity:", leftOutTake.getVelocity());
        */
        sleep(seconds * 1000);

        //launch the last ring
        leftOutTake.setPower(-outtakePower);
        intakeWheel.setPower(1);
        conveyor.setPower(-conveyorPower);
        telemetry.addData("velocity:", leftOutTake.getVelocity());
        sleep(1500);
    }

    public static double clip(double val, double max, double min) {
        int sign;
        if (val < 0) {
            sign = -1;
        } else {
            sign = 1;
        }
        if (Math.abs(val) < min) {
            return min * sign;
        } else if (Math.abs(val) > max) {
            return max * sign;
        } else {
            return val;
        }
    }

    // Movement and Turning methods

    public void move(double forwardinches, double leftinches) {
        resetDriveMotors();
        int forward = (int) inchesToTicks(forwardinches);
        int left = (int) inchesToTicks(leftinches);
        int tolerance = (int) inchesToTicks(0.8);

        double minimumSpeed = 0.05;
        double maximumSpeed = 0.3; //TODO: make this value higher

        double diag1Speed, diag2Speed;
        int diag1Pos, diag2Pos;
        int diag1Error = Integer.MAX_VALUE, diag2Error = Integer.MAX_VALUE;
        int diag1Target = forward - left;
        int diag2Target = forward + left;
        int olddiag1Error = diag1Target, olddiag2Error = diag2Target;

        while (opModeIsActive() && (Math.abs(diag1Error) > tolerance || Math.abs(diag2Error) > tolerance)) {
            diag1Pos = (frontLeft.getCurrentPosition() + backRight.getCurrentPosition());
            diag2Pos = (frontRight.getCurrentPosition() + backLeft.getCurrentPosition());

            diag1Error = diag1Target - diag1Pos;
            diag2Error = diag2Target - diag2Pos;

            diag1Speed = diag1Controller.getPower(diag1Error);
            diag2Speed = diag2Controller.getPower(diag2Error);

            diag1Speed = clip(diag1Speed, maximumSpeed, minimumSpeed);
            diag2Speed = clip(diag2Speed, maximumSpeed, minimumSpeed);

            frontLeft.setPower(diag1Speed);
            backRight.setPower(diag1Speed);
            frontRight.setPower(diag2Speed);
            backLeft.setPower(diag2Speed);

        }
        stopDriveMotors();
        diag1Controller.reset();
        diag2Controller.reset();
    }

    public void turnRelative(double targetAngle) {
        updateIMU();
        turnAbsolute(AngleUnit.normalizeDegrees(targetAngle + currentAngle));
    }

    public void turnAbsolute(double targetAngle) {

        int direction;
        double turnRate;
        double minSpeed = 0.2;
        double maxSpeed = 0.6;
        double tolerance = 1;
        double error = Double.MAX_VALUE;
        double P = 1d / 154;

        while (opModeIsActive() && (Math.abs(error) > tolerance)) {
            updateIMU();
            error = getAngleDist(currentAngle, targetAngle);
            direction = getAngleDir(currentAngle, targetAngle);
            turnRate = clip(P * error, maxSpeed, minSpeed)*direction;
            telemetry.addData("error", error);
            telemetry.addData("turnRate", turnRate);
            telemetry.addData("current", currentAngle);
            telemetry.addData("dir", direction);
            telemetry.update();

            frontLeft.setPower(turnRate);
            backLeft.setPower(turnRate);
            frontRight.setPower(-turnRate);
            backRight.setPower(-turnRate);
        }
        stopDriveMotors();
    }

    private double getAngleDist(double targetAngle, double currentAngle) {
        double angleDifference = currentAngle - targetAngle;

        if (Math.abs(angleDifference) > 180) {
            angleDifference = 360 - Math.abs(angleDifference);
        } else {
            angleDifference = Math.abs(angleDifference);
        }

        return angleDifference;
    }


    private int getAngleDir(double targetAngle, double currentAngle) {
        double angleDifference = targetAngle - currentAngle;
        int angleDir = (int) (angleDifference / Math.abs(angleDifference));

        if (Math.abs(angleDifference) > 180) {
            angleDir *= -1;
        }

        return angleDir;
    }


    /*
    public void moveRelative(double sideways, double forward) {
        moveAndTurn(sideways, forward, 0);
    }

    public void turnRelative(double degrees) {
        moveAndTurn(0, 0, degrees - currentAngle);
    }

    public void turn(double degrees) {
        moveAndTurn(0, 0, degrees);
    }
*/
   /* public void moveAndTurn(double xTarget, double yTarget, double angleError) {

        resetDriveMotors();

        double xError = inchesToTicks(xTarget);

        double yError = inchesToTicks(yTarget);
        /*
         PIDController xDirection = new PIDController(0.05F,0.035F,0.014F);
         PIDController yDirection = new PIDController(0.05F,0.035F,0.014F);
         PIDController angle = new PIDController(0.05F,0.035F,0.014F);
         xDirection.start();
         yDirection.start();
         angle.start();

        double prevXError = inchesToTicks(xTarget);
        double prevYError = inchesToTicks(yTarget);

        //tune the PID and tolerance here
        double P = 0.05;
        double I = 0.035;
        double D = 0.014;
        double tolerance = 4; //(ticks)

        double angleP = 0.02;
        double angleI = 0.01;
        double angleD = 0.0;

        double prevAngleError = 0.0;
        double yIntegral = 0.0;
        double xIntegral = 0.0;
        double aIntegral = 0.0;
        ElapsedTime timer = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);
        double lastTime = 0.0;

        while (opModeIsActive() && (Math.abs(yError) > tolerance || Math.abs(xError) > tolerance)) {
            double currentTime = timer.milliseconds();

            /*
            double yProportion = P * yError;
            double xProportion = P * xError;
            double aProportion = angleP * angleError;
             */
/*
            yIntegral += (yError * (currentTime - lastTime));
            xIntegral += (xError * (currentTime - lastTime));
            aIntegral += (angleError * (currentTime - lastTime));

            double yDerivative = (yError - prevYError) / (currentTime);
            double xDerivative = (xError - prevXError) / (currentTime);
            double aDerivative = (angleError - prevAngleError) / (currentTime);

            currentTime = lastTime;
            yError = prevYError;
            xError = prevXError;
            prevAngleError = angleError;

            double frontRightPower = (P * yError + I * yIntegral - D * yDerivative) +
                    (P * xError + I * xIntegral + D * xDerivative);
            double frontLeftPower = (P * yError + I * yIntegral - D * yDerivative) -
                    (P * xError + I * xIntegral + D * xDerivative);

            double rightTurn = angleP * angleError + angleI * aIntegral - angleD * aDerivative;
            double leftTurn = -angleP * angleError - angleI * aIntegral + angleD * aDerivative;

            double negTarget = yTarget - xTarget;
            double posTarget = yTarget + xTarget;

            double currentYPos = frontLeft.getCurrentPosition();
            double currentXPos = frontRight.getCurrentPosition();

            yError = negTarget - currentYPos;
            xError = posTarget - currentXPos;




            frontRight.setPower(frontRightPower + rightTurn);
            backLeft.setPower(frontRightPower + leftTurn);

            frontLeft.setPower(frontLeftPower + leftTurn);
            backRight.setPower(frontLeftPower + rightTurn);



            telemetry.addData("P-value:", P);
            telemetry.addData("I-value:", I);
            telemetry.addData("D-value:", D);
            telemetry.addData("xError", xError);
            telemetry.addData("yError", yError);
            telemetry.addData("prevXError", prevXError);
            telemetry.addData("prevYError", prevYError);
            telemetry.addData("current x pos", currentXPos);
            telemetry.addData("current y pos", currentYPos);
            telemetry.addData("frontRightPower", frontRightPower);
            telemetry.addData("frontLeftPower", frontLeftPower);
            telemetry.addData("rightTurn", rightTurn);
            telemetry.addData("leftTurn", leftTurn);
            telemetry.update();
            updateIMU();
        }
        stopDriveMotors();
    }
*/
    public double ticksToInches(int ticks) {
        return ticksPerInch / ticks;
    }

    public double inchesToTicks(double inches) {
        return (int) (inches / ticksPerInch);
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

    private void resetOrientation() {
        initialHeading = orientation.firstAngle;
    }

    public void setPosition(double XPos, double YPos) {
        this.XPos = XPos;
        this.YPos = YPos;
    }

    public void setBasicTolerance(int ticks) {
        backLeft.setTargetPositionTolerance(ticks);
        backRight.setTargetPositionTolerance(ticks);
        frontLeft.setTargetPositionTolerance(ticks);
        frontRight.setTargetPositionTolerance(ticks);
    }

    public void driveStraightBasic(double power, long seconds) throws InterruptedException {
        frontLeft.setPower(power);
        frontRight.setPower(power);
        backLeft.setPower(power);
        backRight.setPower(power);
        sleep(1000 * seconds);
    }

    public void driveStraight(double inches, double power) {
        frontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        resetDriveMotors();

        frontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        frontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        setBasicTolerance(4);

        int ticks = (int) (ticksPerInch * inches);

        backLeft.setTargetPosition(ticks + backLeft.getCurrentPosition());
        backRight.setTargetPosition(ticks + backRight.getCurrentPosition());
        frontLeft.setTargetPosition(ticks + frontLeft.getCurrentPosition());
        frontRight.setTargetPosition(ticks + frontRight.getCurrentPosition());
        backLeft.setPower(power);
        backRight.setPower(power);
        frontLeft.setPower(power);
        frontRight.setPower(power);
        while (motorsBusy() && opModeIsActive()) {
            telemetry.addData("frontLeft:", frontLeft.getCurrentPosition());
            telemetry.addData("frontRight:", frontRight.getCurrentPosition());
            telemetry.addData("backLeft:", backLeft.getCurrentPosition());
            telemetry.addData("backRight:", backRight.getCurrentPosition());
            telemetry.update();
        }
        backLeft.setPower(0);
        backRight.setPower(0);
        frontLeft.setPower(0);
        frontRight.setPower(0);
        resetDriveMotors();
    }

    public void rotate(double degrees) {
        double factor = degrees * (ticksPerInch / 180);

        backLeft.setTargetPosition((int) factor);
        backRight.setTargetPosition(-(int) factor);
        frontLeft.setTargetPosition((int) factor);
        frontRight.setTargetPosition(-(int) factor);
        setBasicTolerance(4);
        setRunToPosition();
        while (motorsBusy()) {
            backLeft.setPower(1);
            backRight.setPower(1);
            frontLeft.setPower(1);
            frontRight.setPower(1);
            telemetry.addData("Target position:", frontLeft.getTargetPositionTolerance());
            telemetry.update();
        }
        resetDriveMotors();
    }
    // AUTONOMOUS METHODS

    // Autonomous Paths

    public void moveForwardAndLaunch() throws InterruptedException {
        turnRelative(-90); //turn so that robot faces forward
        move(58, 0); //move behind launch line
        doEverything(0.73, 0.4, 5);
    }

    public void launchAndPark() throws InterruptedException{
        doEverything(0.73, 0.4, 5); //launch rings
        //move(12,0); //park on launch line
        telemetry.addData("launch and park?", "yes");
    }

    public void backUpAuto() throws InterruptedException {
        moveForwardAndLaunch();
        move(12, 0); //move forward to park
    }

    public void targetZoneARed() throws InterruptedException {
        turnRelative(-90); //turn so that robot faces forward
        move(60,0); //drive to target zone A
        move(0,-24); //strafe right to target zone A
        turnRelative(180);
        autoControlWobbleArm(); //drop wobble goal
        move(10, 0); //move "back" behind launch line
        turnRelative(180);
        sleep(500);
        //launchAndPark();
    }

    public void targetZoneBRed() throws InterruptedException {
        turnRelative(-90); //turn so that robot faces forward
        move(80,-24); //drive to target zone B
        turnRelative(180);
        autoControlWobbleArm(); //drop wobble goal
        move(24, 0); //move "back" behind launch line
        turnRelative(180);
        launchAndPark();
    }

    public void targetZoneCRed() throws InterruptedException {
        turnRelative(-90); //turn so that robot faces forward
        move(120,-24); //drive to target zone B
        turnRelative(180);
        controlWobbleArm(); //drop wobble goal
        move(64, 0); //move "back" behind launch line
        turnRelative(180);
        launchAndPark();
    }

/*
    public int detectRings() { return 0;
*/
/*
            int targetZone = 0;

            try {
                initTfod();
            } catch (Exception e){
                e.printStackTrace();
            }

            try {
                initVuforia();
            } catch (Exception e){
                e.printStackTrace();
            }
 *//*           initTfod();
            initVuforia();*//*

            if (tfod != null) {
                tfod.activate();
            }

            telemetry.update();
            waitForStart();

            ElapsedTime timer = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);
            timer.reset();

            if (opModeIsActive()) {
                while (opModeIsActive()) {
                    if (tfod != null) {
                        // getUpdatedRecognitions() will return null if no new information is available since
                        // the last time that call was made.
                        List<Recognition> updatedRecognitions = tfod.getUpdatedRecognitions();

                        while (timer.time() < 3000) {
                            if (updatedRecognitions != null) {
                                telemetry.addData("# Object Detected", updatedRecognitions.size());
                                if (updatedRecognitions.size() == 0) {
                                    // empty list.  no objects recognized.
                                    telemetry.addData("TFOD", "No items detected.");
                                    telemetry.addData("Target Zone", "A");
                                    targetZone = (int) TARGET_A;
                                } else {
                                    // list is not empty.
                                    // step through the list of recognitions and display boundary info.
                                    int i = 0;
                                    for (Recognition recognition : updatedRecognitions) {
                                        telemetry.addData(String.format("label (%d)", i), recognition.getLabel());
                                        telemetry.addData(String.format("  left,top (%d)", i), "%.03f , %.03f",
                                                recognition.getLeft(), recognition.getTop());
                                        telemetry.addData(String.format("  right,bottom (%d)", i), "%.03f , %.03f",
                                                recognition.getRight(), recognition.getBottom());

                                        // check label to see which target zone to go after.
                                        if (recognition.getLabel().equals("Single")) {
                                            telemetry.addData("Target Zone", "B");
                                            targetZone = (int) TARGET_B;
                                        } else if (recognition.getLabel().equals("Quad")) {
                                            telemetry.addData("Target Zone", "C");
                                            targetZone = (int) TARGET_C;
                                        } else {
                                            telemetry.addData("Target Zone", "UNKNOWN");
                                        }
                                    }
                                }
                                telemetry.update();
                            }
                        }
                        timer.reset();
                    }
                }
            }

            if (tfod != null) {
                tfod.shutdown();
            }

            return targetZone;
        }*/

    /*    public void moveToTargetALeft (String color) throws InterruptedException {
            waitForStart();
            moveRelative(0, 70); //Move towards square A
            turnRelative(-30); //Turn facing square A
            moveRelative(0, 15); //Move into square A
            turnRelative(-330); //Turn shooter towards ring goal
            moveRelative(0, 80); //Move back behind white line
            launch(2, 3); //Shoot rings
            moveRelative(0, -5); //Back into white line
        }

        public void moveToTargetBLeft (String color) throws InterruptedException {
            waitForStart();
            moveRelative(0, 30); //Moves forward into white line
            turnRelative(30); //Turn towards square B
            moveRelative(0, 20); //Moves into square B
            turnRelative(330); //Turn so back is facing ring Goal
            moveRelative(0, 20); //Head back behind white line
            launch(2, 3); //Shoot rings
            moveRelative(0, -10); //Park onto white line
        }

        public void moveToTargetCLeft (String color) throws InterruptedException {
            moveRelative(0, 40); //Move towards square C
            turnRelative(-30); //Turn towards square C
            moveRelative(0, 10); //Move into square C
            turnRelative(-330); //Turn shooter towards ring goal
            moveRelative(0, 10); //Move behind white line
            launch(2, 3); //Shoot the rings
            moveRelative(0, -5); //Park on white line
        }

        // robot starts on right side
        public void moveToTargetARight (String color) throws InterruptedException {
            waitForStart();
            moveRelative(0, 70); //Move towards square A
            turnRelative(-45); //Turn facing square A
            moveRelative(0, 15); //Move into square A
            turnRelative(-315); //Turn shooter towards ring goal
            moveRelative(0, 80); //Move back behind white line
            launch(2, 3); //Shoot rings
            moveRelative(0, -5); //Back into white line
        }

        public void moveToTargetBRight (String color) throws InterruptedException {
            waitForStart();
            moveRelative(0, 30); //Moves forward into white line
            turnRelative(-30); //Turn towards square B
            moveRelative(0, 20); //Moves into square B
            turnRelative(-330); //Turn so back is facing ring Goal
            moveRelative(0, 20); //Head back behind white line
            launch(2, 3); //Shoot rings
            moveRelative(0, -10); //Park onto white line
        }

        public void moveToTargetCRight (String color) throws InterruptedException {
            moveRelative(0, 40); //Move towards square C
            moveRelative(0, 40); //Move towards square C
            turnRelative(-45); //Turn towards square C
            moveRelative(0, 17); //Move into square C
            turnRelative(-315); //Turn shooter towards ring goal
            moveRelative(0, 10); //Move behind white line
            launch(2, 3); //Shoot the rings
            moveRelative(0, -5); //Park on white line
        }

        /**
         * Initialize the Vuforia localization engine.
         *
        private void initVuforia () {
            /*
             * Configure Vuforia by creating a Parameter object, and passing it to the Vuforia engine.
             *
            VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();

            parameters.vuforiaLicenseKey = VUFORIA_KEY;
            parameters.cameraDirection = VuforiaLocalizer.CameraDirection.BACK;

            //  Instantiate the Vuforia engine
            vuforia = ClassFactory.getInstance().createVuforia(parameters);

            // Loading trackables is not necessary for the TensorFlow Object Detection engine.
        }

        /**
         * Initialize the TensorFlow Object Detection engine.
         *
        private void initTfod () {
            int tfodMonitorViewId = hardwareMap.appContext.getResources().getIdentifier(
                    "tfodMonitorViewId", "id", hardwareMap.appContext.getPackageName());
            TFObjectDetector.Parameters tfodParameters = new TFObjectDetector.Parameters(tfodMonitorViewId);
            tfodParameters.minResultConfidence = 0.8f;
            tfod = ClassFactory.getInstance().createTFObjectDetector(tfodParameters, vuforia);
            tfod.loadModelFromAsset(TFOD_MODEL_ASSET, LABEL_FIRST_ELEMENT, LABEL_SECOND_ELEMENT);
        }*/
    }




