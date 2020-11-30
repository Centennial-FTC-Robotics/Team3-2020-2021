package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.DcMotor;

public class VortechsHardware extends LinearOpMode {

    //initialize motors and servos here
    protected DcMotorEx frontRight;
    protected DcMotorEx frontLeft;
    protected DcMotorEx backRight;
    protected DcMotorEx backLeft;
    protected DcMotorEx leftOutTake;
    protected DcMotorEx rightOutTake;
    protected DcMotorEx intakeWheel;


    @Override
    public void runOpMode() throws InterruptedException {

        frontRight = hardwareMap.get(DcMotorEx.class, "frontRight");
        frontLeft = hardwareMap.get(DcMotorEx.class, "frontLeft");
        backRight = hardwareMap.get(DcMotorEx.class, "backRight");
        backLeft = hardwareMap.get(DcMotorEx.class, "backLeft");
        leftOutTake = hardwareMap.get(DcMotorEx.class, "leftOutTake");
        rightOutTake = hardwareMap.get(DcMotorEx.class, "rightOutTake");
        intakeWheel = hardwareMap.get(DcMotorEx.class, "intakeWheel");


    }
}
