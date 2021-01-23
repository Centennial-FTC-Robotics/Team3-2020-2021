package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;

import java.util.List;

public class ObjectDetector extends AutoPaths {
    public TFObjectDetector tfod;
    public VuforiaLocalizer vuforia;
    private static final String TFOD_MODEL_ASSET = "UltimateGoal.tflite";
    private static final String LABEL_FIRST_ELEMENT = "Quad";
    private static final String LABEL_SECOND_ELEMENT = "Single";
    public OpMode opMode;

    public void runOpMode() throws InterruptedException {
        initVuforia();
        initTfod();
        if(tfod!=null){
            tfod.activate();
        }
        moveToCorrectTarget();
    }

    public void initVuforia() {
        //create parameter object and pass it to create Vuforia engine
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();
        parameters.vuforiaLicenseKey = "AaE18sD/////AAABmZ7zTjrwDEwgoUUd9Hg/fVNlCi1mnUJCizFmysoKuVPPNnIEWJmK9SlpRppNs0SV9sDdCFc6nySaX1KM3CimlwDwzEcmZs016lHBxh3A0S5hVFPPHWzE34TCYgA90g9nrKrwRIFolSSO6p9YmDLzi4fFHcOe85nuiYRfFZwaYlCnTZnwU3czaUue9uFiq3Q9e9Hytr3EtxJrvKISSdNah+WP+43QaqrLcQR7NfOkYQ5AY+omdtZ76KfgooK5dtO4lgYwxAWkVVAYt60zLcrpd4ZHC9Nu+6xkhLF5QhJDbfSUD0/Kep5MhZqugCpguNDzvcBQ5HtCVYvjGYO6pe7Gy6JwRiB1E2gAatjyS0Prc2pV";
        parameters.cameraDirection = VuforiaLocalizer.CameraDirection.BACK;

        vuforia = ClassFactory.getInstance().createVuforia(parameters);
    }

    public void initTfod() {
        int tfodMonitorViewId = opMode.hardwareMap.appContext.getResources().getIdentifier(
                "tfodMonitorViewId", "id", opMode.hardwareMap.appContext.getPackageName());
        TFObjectDetector.Parameters tfodParameters = new TFObjectDetector.Parameters(tfodMonitorViewId);
        tfodParameters.minimumConfidence = 0.8;
        tfod = ClassFactory.getInstance().createTFObjectDetector(tfodParameters, vuforia);
        tfod.loadModelFromAsset(TFOD_MODEL_ASSET, LABEL_FIRST_ELEMENT, LABEL_SECOND_ELEMENT);

    }

    //move to correct target zone based on number of rings
    //not 100% sure if this actually works
    public void moveToCorrectTarget() throws InterruptedException {
        List<Recognition> updatedRecognitions = tfod.getRecognitions();
        if (updatedRecognitions != null) {
            for (Recognition recognition : updatedRecognitions) {
                opMode.telemetry.addData("# Rings Detected", updatedRecognitions.size());
                opMode.telemetry.addData(String.format("label (%d)", 0), recognition.getLabel());
                opMode.telemetry.addData(String.format("  left,top (%d)", 0), "%.03f , %.03f",
                        recognition.getLeft(), recognition.getTop());
                opMode.telemetry.addData(String.format("  right,bottom (%d)", 0), "%.03f , %.03f",
                        recognition.getRight(), recognition.getBottom());
                if (recognition.getLabel().equals(LABEL_FIRST_ELEMENT)) {
                    //move to target B, 1 ring
                    moveToTargetB();
                } else if (recognition.getLabel().equals(LABEL_SECOND_ELEMENT)) {
                    //move to target C, 4 rings
                    moveToTargetC();
                } else {
                    //move to target A, 0 rings
                    moveToTargetA();
                }
            }
        }
    }

}

