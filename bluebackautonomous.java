//test this code! if it doesn't work go back to the old one
// should: 1) move forward 2) rotate until it detects the apriltag 3) align to the apriltag
//will want to get the game field set up first before doing

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

@Autonomous(name = "BlueBackAutonomous.java")
public class BlueBackAutonomous extends LinearOpMode {

    private Blinker control_Hub;
    private DcMotor bottomL = null;
    private DcMotor bottomR = null;
    private DcMotor topL = null;
    private DcMotor topR = null;
    
    private static final boolean USE_WEBCAM = true; // true if using Webcam 1
    private static final int[] DESIRED_TAG_IDS = {21, 22, 23};
    private VisionPortal visionPortal;
    private AprilTagProcessor aprilTag;
    private int seenTagId = -1;

    private final double DESIRED_DISTANCE_INCHES = 12.0;
    private final double SPEED_GAIN = 0.02;
    private final double TURN_GAIN = 0.01;
    private final double MAX_AUTO_SPEED = 0.5;
    private final double MAX_AUTO_TURN = 0.35;
    private final double RANGE_TOLERANCE = 1.5;   // inches
    private final double BEARING_TOLERANCE = 4.0; // degrees

    private final ElapsedTime runtime = new ElapsedTime();

    @Override
@Override
public void runOpMode() {
    // Initialize hardware
    topL = hardwareMap.get(DcMotor.class, "topL");
    topR = hardwareMap.get(DcMotor.class, "topR");
    bottomL = hardwareMap.get(DcMotor.class, "bottomL");
    bottomR = hardwareMap.get(DcMotor.class, "bottomR");

    bottomL.setDirection(DcMotor.Direction.REVERSE);
    topL.setDirection(DcMotor.Direction.REVERSE);
    topR.setDirection(DcMotor.Direction.FORWARD);

    topL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    topR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    bottomL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    bottomR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

    topL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    topR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    bottomL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    bottomR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

    initAprilTag();
    if (USE_WEBCAM) setManualExposure(6, 250);

    telemetry.addLine("Initialized. Scanning for desired tags pre-start...");
    telemetry.update();
    sampleTagBeforeStart(1.5);

    telemetry.addData("Prestart seenTagId", seenTagId >= 0 ? seenTagId : "none");
    telemetry.addLine("Press PLAY to start autonomous.");
    telemetry.update();

    waitForStart();
    runtime.reset();

    if (!opModeIsActive()) return;

    // --- 1. Drive forward while scanning ---
    driveForward(25, 0.5); // Move forward a safe distance

// ---------- Replace the search loop (inside runOpMode) with this ----------
if (seenTagId < 0 || !isDesiredTag(seenTagId)) {
    telemetry.addLine("No tag seen prestart — scanning now...");
    telemetry.update();

    long searchStart = System.currentTimeMillis();
    long searchTimeoutMs = 15000; // 15s scan
    final double TURN_SEARCH_POWER = 0.12; // try 0.12; increase to 0.18-0.20 if too slow

    // Continuous-sweep approach: keep rotating until we see a desired tag or timeout
    while (opModeIsActive() && (System.currentTimeMillis() - searchStart) < searchTimeoutMs
            && !isDesiredTag(seenTagId)) {

        List<AprilTagDetection> detections = aprilTag.getDetections();

        if (detections != null && !detections.isEmpty()) {
            // check each detection for desired id
            for (AprilTagDetection d : detections) {
                if (d != null && isDesiredTag(d.id)) {
                    seenTagId = d.id;
                    telemetry.addData("Tag found", seenTagId);
                    telemetry.update();
                    break;
                }
            }

            // if we found a tag, break the loop; otherwise keep rotating
            if (isDesiredTag(seenTagId)) {
                break;
            } else {
                // give the pipeline a little time to process frames while rotating
                setMecanumPower(0.0, 0.0, TURN_SEARCH_POWER);
                sleep(120);
            }
        } else {
            // No detections → keep rotating continuously (don't stop every short pulse)
            setMecanumPower(0.0, 0.0, TURN_SEARCH_POWER);
            telemetry.addLine("Rotating to search...");
            telemetry.update();
            // sleep a bit to let the motion happen and the vision pipeline update
            sleep(120);
        }
    }
    stopAllDrivePower();
}


    // --- 3. Center on the tag ---
    if (seenTagId >= 0 && isDesiredTag(seenTagId)) {
        telemetry.addData("Tag to center on", seenTagId);
        telemetry.update();

        boolean centered = alignToTagAndCenter(seenTagId, 6000); // 6s timeout
        telemetry.addData("Centering result", centered ? "Centered" : "Timed out");
        telemetry.update();

        // --- 4. Execute corresponding action ---
        if (centered) {
            if (seenTagId == 21) moveGpp();
            else if (seenTagId == 22) movePgp();
            else if (seenTagId == 23) movePpg();
        } else {
            telemetry.addLine("Centering failed — skipping action");
            telemetry.update();
        }
    } else {
        telemetry.addLine("No desired tag detected — autonomous ends");
        telemetry.update();
    }

    stopAllDrivePower();
    telemetry.addLine("Autonomous complete");
    telemetry.update();
}

    private void moveGpp() {
        telemetry.addData("gpp", "called");
        telemetry.update();
    }
    
    private void movePpg() {
        telemetry.addData("ppg", "called");
        telemetry.update();
    }
    
    private void movePgp() {
        telemetry.addData("pgp", "called");
        telemetry.update();
    }

private boolean alignToTagAndCenter(int targetId, long timeoutMs) {
    long start = System.currentTimeMillis();

    final double CENTER_TOLERANCE_X = 20; // pixels left/right
    final double TURN_GAIN = 0.01;        // heading adjustment
    final double IMAGE_WIDTH = 1280;
    final double IMAGE_CENTER_X = IMAGE_WIDTH / 2.0;

    while (opModeIsActive() && (System.currentTimeMillis() - start) < timeoutMs) {
        List<AprilTagDetection> detections = aprilTag.getDetections();
        AprilTagDetection found = null;

        // Only consider the tag we’re centering on
        for (AprilTagDetection d : detections) {
            if (d.metadata != null && d.id == targetId) {
                found = d;
                break;
            }
        }

        if (found == null) {
            // Tag temporarily lost: stop motors and continue waiting
            stopAllDrivePower();
            telemetry.addLine("Tag lost temporarily...");
            telemetry.update();
            sleep(50);
            continue;
        }

        // Horizontal pixel offset
        double errorX = found.getCenterX() - IMAGE_CENTER_X; 
        double turn = -found.ftcPose.bearing * TURN_GAIN;

        // Only strafe + turn to center horizontally
        double strafe = -errorX * 0.005; 

        setMecanumPower(0.0, strafe, turn);

        telemetry.addData("ErrorX", errorX);
        telemetry.addData("Bearing", found.ftcPose.bearing);
        telemetry.update();

        if (Math.abs(errorX) <= CENTER_TOLERANCE_X && Math.abs(found.ftcPose.bearing) <= BEARING_TOLERANCE) {
            stopAllDrivePower();
            return true;
        }

        sleep(40);
    }

    stopAllDrivePower();
    return false;
}

    
    private void setMecanumPower(double forward, double strafe, double rotate) {
        double fl = forward + strafe + rotate;
        double fr = forward - strafe - rotate;
        double bl = forward - strafe + rotate;
        double br = forward + strafe - rotate;
        double max = Math.max(Math.max(Math.abs(fl), Math.abs(fr)), Math.max(Math.abs(bl), Math.abs(br)));
        if (max > 1.0) {
            fl /= max;
            fr /= max;
            bl /= max;
            br /= max;
        }

        topL.setPower(fl);
        topR.setPower(fr);
        bottomL.setPower(bl);
        bottomR.setPower(br);
    }
    
    // Method to drive forward or backward a given distance (in inches)
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

        while (opModeIsActive() && topL.isBusy() && topR.isBusy() && bottomL.isBusy() && bottomR.isBusy()) {
            telemetry.addData("Path", "Driving Forward");
            telemetry.update();
        }

        stopMotors();
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

        while (opModeIsActive() && topL.isBusy() && topR.isBusy() && bottomL.isBusy() && bottomR.isBusy()) {
            telemetry.addData("Path", "Turning Left");
            telemetry.update();
        }

        stopMotors();
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

        while (opModeIsActive() && topL.isBusy() && topR.isBusy() && bottomL.isBusy() && bottomR.isBusy()) {
            telemetry.addData("Path", "Turning Right");
            telemetry.update();
        }

        stopMotors();
    }

    // Method to stop all motors
    private void stopMotors() {
        topL.setPower(0);
        topR.setPower(0);
        bottomL.setPower(0);
        bottomR.setPower(0);

        topL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        topR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        bottomL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        bottomR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
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
        final double ROBOT_DIAMETER = 18.0;  // Example robot diameter in inches
        final double ROBOT_CIRCUMFERENCE = Math.PI * ROBOT_DIAMETER;
        double distance = (degrees / 360.0) * ROBOT_CIRCUMFERENCE;
        return inchesToTicks(distance);
    }

   // -----------------------
    // VISION helpers
    // -----------------------
    private void initAprilTag() {
        aprilTag = new AprilTagProcessor.Builder().build();
        aprilTag.setDecimation(2);

        if (USE_WEBCAM) {
            visionPortal = new VisionPortal.Builder()
                    .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"))
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
                telemetry.addData("preseen", "%d (%s)", det.id, det.metadata != null ? det.metadata.name : "no-meta");
                if (det.metadata != null && isDesiredTag(det.id)) {
                    seenTagId = det.id;
                    telemetry.addData("prechosen", seenTagId);
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

    private void stopAllDrivePower() {
        setMecanumPower(0.0, 0.0, 0.0);
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
}
