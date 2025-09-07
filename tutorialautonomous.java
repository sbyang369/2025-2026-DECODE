//this is the final tutorial code that scans an apriltag, centers it, then goes to the set location

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.Blinker;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.ExposureControl;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.GainControl;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;
import java.util.concurrent.TimeUnit;

@Autonomous(name = "BlueBack_AprilTag_Autonomous")
public class BlueBack_AprilTag_Autonomous extends LinearOpMode {

    private DcMotor topL = null;
    private DcMotor topR = null;
    private DcMotor bottomL = null;
    private DcMotor bottomR = null;
  
    private static final boolean USE_WEBCAM = true;  
    private static final int[] DESIRED_TAG_IDS = {21, 22, 23};
    private VisionPortal visionPortal;
    private AprilTagProcessor aprilTag;
    private int seenTagId = -1;   // stores the ID (21/22/23) when found pre-start or during run


    private final double DESIRED_DISTANCE_INCHES = 12.0;  // how close you want to be to the tag
    private final double SPEED_GAIN = 0.02;   // proportional gain for forward/back control (tune)
    private final double TURN_GAIN = 0.01;    // proportional gain for heading control (tune)
    private final double MAX_AUTO_SPEED = 0.5;
    private final double MAX_AUTO_TURN = 0.35;
    private final double RANGE_TOLERANCE = 1.5;   // inches
    private final double BEARING_TOLERANCE = 4.0; // degrees

    private final ElapsedTime runtime = new ElapsedTime();

    @Override
    public void runOpMode() {
        topL = hardwareMap.get(DcMotor.class, "topL");
        topR = hardwareMap.get(DcMotor.class, "topR");
        bottomL = hardwareMap.get(DcMotor.class, "bottomL");
        bottomR = hardwareMap.get(DcMotor.class, "bottomR");
        bottomL.setDirection(DcMotor.Direction.REVERSE);
        topL.setDirection(DcMotor.Direction.REVERSE);
        // topR left forward? had topR reversed earlier; verify on bot
        topR.setDirection(DcMotor.Direction.FORWARD);

        topL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        topR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        bottomL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        bottomR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        topL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        topR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        bottomL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        bottomR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        topL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        topR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        bottomL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        bottomR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        initAprilTag();
      
        if (USE_WEBCAM) setManualExposure(6, 250);

        telemetry.addData("Status", "Initialized. Looking for tags pre-start...");
        telemetry.update();

        sampleTagBeforeStart(1.5);

        telemetry.addData("Prestart tag", seenTagId >= 0 ? seenTagId : "none");
        telemetry.addData(">", "Press PLAY");
        telemetry.update();

        waitForStart();
        runtime.reset();

        if (opModeIsActive()) {
            if (seenTagId < 0 || !isDesiredTag(seenTagId)) {
                seenTagId = -1;               // be explicit: clear any non-desired ID
                sampleTagBeforeStart(1.0);    // brief attempt to find a desired tag
            }

            if (seenTagId >= 0 && isDesiredTag(seenTagId)) {
                telemetry.addData("Tag chosen", seenTagId);
                telemetry.update();

                boolean aligned = alignToTagAndCenter(seenTagId, 5000); // 5000 ms timeout

                telemetry.addData("Aligned", aligned);
                telemetry.update();

              if (seenTagId == 21) {
                    driveToGPP();
                } else if (seenTagId == 22) {
                    driveToID22();
                } else if (seenTagId == 23) {
                    driveToID23();
                }
            } else {
                telemetry.addData("No desired tag", "found - running default behavior");
                telemetry.update();
                // fallback code here 
            }
        }
    }

    // -----------------------
    // VISION / APRILTAG UTIL
    // -----------------------
    private void initAprilTag() {
        aprilTag = new AprilTagProcessor.Builder().build();
        aprilTag.setDecimation(2);

        if (USE_WEBCAM) {
            visionPortal = new VisionPortal.Builder()
                    .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1")) // change name if needed
                    .addProcessor(aprilTag)
                    .build();
        } else {
            visionPortal = new VisionPortal.Builder()
                    .setCamera(BuiltinCameraDirection.BACK)
                    .addProcessor(aprilTag)
                    .build();
        }
    }

    private void sampleTagBeforeStart(double timeoutSeconds) {
        double start = runtime.seconds();
        while (!isStarted() && !isStopRequested() && (runtime.seconds() - start) < timeoutSeconds) {
            List<AprilTagDetection> detections = aprilTag.getDetections();
            for (AprilTagDetection det : detections) {
                telemetry.addData("Seen tag", "%d (%s)", det.id, det.metadata != null ? det.metadata.name : "no-metadata");
                if (det.metadata != null && isDesiredTag(det.id)) {
                    seenTagId = det.id;
                    telemetry.addData("Chosen", seenTagId);
                    telemetry.update();
                    return;
                }
            }
            telemetry.update();
            sleep(50);
        }
    }

    private boolean isDesiredTag(int id) {
        for (int x : DESIRED_TAG_IDS) if (x == id) return true;
        return false;
    }
    private boolean alignToTagAndCenter(int targetId, long timeoutMs) {
        long start = System.currentTimeMillis();
        while (opModeIsActive() && (System.currentTimeMillis() - start) < timeoutMs) {

            List<AprilTagDetection> detections = aprilTag.getDetections();
            AprilTagDetection targetDetection = null;
            for (AprilTagDetection d : detections) {
                if (d.metadata != null && d.id == targetId && isDesiredTag(d.id)) {
                    targetDetection = d;
                    break;
                }
            }

            if (targetDetection == null) {
                setTankPower(0.10, -0.10); // small rotation
                telemetry.addData("Searching", "for tag %d", targetId);
                telemetry.update();
                sleep(50);
                continue;
            }

            double range = targetDetection.ftcPose.range;     // inches
            double bearing = targetDetection.ftcPose.bearing; // degrees

            double rangeError = range - DESIRED_DISTANCE_INCHES;
            double headingError = bearing;

            double drive = Range.clip(rangeError * SPEED_GAIN, -MAX_AUTO_SPEED, MAX_AUTO_SPEED);
            double turn = Range.clip(headingError * TURN_GAIN, -MAX_AUTO_TURN, MAX_AUTO_TURN);

            double leftPower = drive - turn;
            double rightPower = drive + turn;

            double max = Math.max(Math.abs(leftPower), Math.abs(rightPower));
            if (max > 1.0) {
                leftPower /= max;
                rightPower /= max;
            }

            setTankPower(leftPower, rightPower);

            telemetry.addData("Tag", targetId);
            telemetry.addData("Range", "%.2f (err %.2f)", range, rangeError);
            telemetry.addData("Bearing", "%.2f (err %.2f)", bearing, headingError);
            telemetry.update();

            if (Math.abs(rangeError) <= RANGE_TOLERANCE && Math.abs(headingError) <= BEARING_TOLERANCE) {
                stopMotorsPower();
                return true;
            }

            sleep(40);
        }

        stopMotorsPower();
        return false;
    }

    private void setTankPower(double left, double right) {
        topL.setPower(left);
        bottomL.setPower(left);
        topR.setPower(right);
        bottomR.setPower(right);
    }

    private void stopMotorsPower() {
        setTankPower(0, 0);
    }

    private void setManualExposure(int exposureMS, int gain) {
        if (visionPortal == null) return;

        if (visionPortal.getCameraState() != VisionPortal.CameraState.STREAMING) {
            telemetry.addData("Camera", "Waiting for stream");
            telemetry.update();
            while (!isStopRequested() && (visionPortal.getCameraState() != VisionPortal.CameraState.STREAMING)) {
                sleep(20);
            }
            telemetry.addData("Camera", "Ready");
            telemetry.update();
        }

        if (!isStopRequested()) {
            ExposureControl exposureControl = visionPortal.getCameraControl(ExposureControl.class);
            if (exposureControl.getMode() != ExposureControl.Mode.Manual) {
                exposureControl.setMode(ExposureControl.Mode.Manual);
                sleep(50);
            }
            exposureControl.setExposure((long) exposureMS, TimeUnit.MILLISECONDS);
            sleep(20);
            GainControl gainControl = visionPortal.getCameraControl(GainControl.class);
            gainControl.setGain(gain);
            sleep(20);
            telemetry.addData("Camera", "Ready");
            telemetry.update();
        }
    }

    // -------------------------
    //  EXISTING MOVEMENT CODE
    public void driveForward(double inches, double power) {
        int ticks = inchesToTicks(inches);

        topL.setTargetPosition(topL.getCurrentPosition() + ticks);
        topR.setTargetPosition(topR.getCurrentPosition() + ticks);
        bottomL.setTargetPosition(bottomL.getCurrentPosition() + ticks);
        bottomR.setTargetPosition(bottomR.getCurrentPosition() + ticks);

        topL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        topR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        bottomL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        bottomR.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        topL.setPower(power);
        topR.setPower(power);
        bottomL.setPower(power);
        bottomR.setPower(power);

        runtime.reset();
        double timeout = Math.max(3.0, Math.abs(inches) / 5.0 + 1.5); // seconds heuristic

        while (opModeIsActive() && topL.isBusy() && topR.isBusy() && bottomL.isBusy() && bottomR.isBusy() && runtime.seconds() < timeout) {
            telemetry.addData("Path", "Driving Forward");
            telemetry.update();
        }

        stopMotorsEncoderMode();
    }

    // Method to turn left a given angle (in degrees)
    public void turnLeft(double degrees, double power) {
        int ticks = degreesToTicks(degrees);

        topL.setTargetPosition(topL.getCurrentPosition() - ticks);
        topR.setTargetPosition(topR.getCurrentPosition() + ticks);
        bottomL.setTargetPosition(bottomL.getCurrentPosition() - ticks);
        bottomR.setTargetPosition(bottomR.getCurrentPosition() + ticks);

        topL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        topR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        bottomL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        bottomR.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        topL.setPower(power);
        topR.setPower(power);
        bottomL.setPower(power);
        bottomR.setPower(power);

        // timeout
        runtime.reset();
        double timeout = Math.max(2.0, Math.abs(degrees) / 90.0 + 1.0);
        while (opModeIsActive() && topL.isBusy() && topR.isBusy() && bottomL.isBusy() && bottomR.isBusy() && runtime.seconds() < timeout) {
            telemetry.addData("Path", "Turning Left");
            telemetry.update();
        }

        stopMotorsEncoderMode();
    }

    // Method to turn right a given angle (in degrees)
    public void turnRight(double degrees, double power) {
        int ticks = degreesToTicks(degrees);

        topL.setTargetPosition(topL.getCurrentPosition() + ticks);
        topR.setTargetPosition(topR.getCurrentPosition() - ticks);
        bottomL.setTargetPosition(bottomL.getCurrentPosition() + ticks);
        bottomR.setTargetPosition(bottomR.getCurrentPosition() - ticks);

        topL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        topR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        bottomL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        bottomR.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        topL.setPower(power);
        topR.setPower(power);
        bottomL.setPower(power);
        bottomR.setPower(power);

        runtime.reset();
        double timeout = Math.max(2.0, Math.abs(degrees) / 90.0 + 1.0);
        while (opModeIsActive() && topL.isBusy() && topR.isBusy() && bottomL.isBusy() && bottomR.isBusy() && runtime.seconds() < timeout) {
            telemetry.addData("Path", "Turning Right");
            telemetry.update();
        }

        stopMotorsEncoderMode();
    }

    // stop and switch back to RUN_USING_ENCODER
    private void stopMotorsEncoderMode() {
        topL.setPower(0);
        topR.setPower(0);
        bottomL.setPower(0);
        bottomR.setPower(0);

        topL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        topR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        bottomL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        bottomR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    // emergency stop (power-based)
    private void stopMotorsPowerAndEncoders() {
        topL.setPower(0);
        topR.setPower(0);
        bottomL.setPower(0);
        bottomR.setPower(0);
    }

    // Helper method to convert inches to encoder ticks
    private int inchesToTicks(double inches) {
        final double TICKS_PER_REV = 537.6;  // Example: GoBILDA 5202 motor
        final double WHEEL_DIAMETER = 4.0;  // In inches
        final double CIRCUMFERENCE = Math.PI * WHEEL_DIAMETER;
        return (int) ((inches / CIRCUMFERENCE) * TICKS_PER_REV);
    }

    // Helper method to convert degrees to encoder ticks
    private int degreesToTicks(double degrees) {
        final double ROBOT_DIAMETER = 18.0;  // Example robot diameter in inches (tune)
        final double ROBOT_CIRCUMFERENCE = Math.PI * ROBOT_DIAMETER;
        double distance = (degrees / 360.0) * ROBOT_CIRCUMFERENCE;
        return inchesToTicks(distance);
    }

    // -----------------------
    // ROUTINES FOR EACH TAG
    // Replace or tune these with your real game actions
    // -----------------------
    private void driveToGPP() {
        // Example sequence for ID 21 (GPP)
        telemetry.addData("DriveTo", "GPP (ID 21)");
        telemetry.update();

        // adjust these to your real routine
        driveForward(10, 0.5);
        turnLeft(45, 0.5);
        driveForward(20, 0.5);
    }

    private void driveToID22() {
        // Example sequence for ID 22 (PGP)
        telemetry.addData("DriveTo", "ID22 (PGP)");
        telemetry.update();

        driveForward(8, 0.5);
        turnRight(30, 0.5);
        driveForward(15, 0.5);
    }

    private void driveToID23() {
        // Example sequence for ID 23 (PPG)
        telemetry.addData("DriveTo", "ID23 (PPG)");
        telemetry.update();

        driveForward(5, 0.5);
        turnLeft(20, 0.5);
        driveForward(25, 0.5);
    }

}
