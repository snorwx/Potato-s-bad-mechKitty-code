package org.firstinspires.ftc.teamcode.pedroPathing;

import android.util.Size;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import org.firstinspires.ftc.vision.opencv.ColorBlobLocatorProcessor;
import org.firstinspires.ftc.vision.opencv.ColorRange;
import org.firstinspires.ftc.vision.opencv.ColorSpace;
import org.firstinspires.ftc.vision.opencv.ImageRegion;
import org.opencv.core.RotatedRect;
import org.opencv.core.Scalar;

import org.firstinspires.ftc.teamcode.Potato_Assets.CameraConstants;

import java.util.List;

/**
 * Quick Camera Test TeleOp
 * Tests webcam functionality with AprilTag and color blob detection.
 * 
 * Camera Config: Uses same "Webcam 1" and settings from old robot
 * 
 * Controls:
 * - DPAD UP: Resume camera stream
 * - DPAD DOWN: Stop camera stream
 * - A: Toggle AprilTag display
 * - B: Toggle Color detection display
 * - X: Cycle through color modes (Yellow/Red/Blue)
 */
@TeleOp(name = "Camera Test", group = "Test")
public class CameraTestTeleOp extends LinearOpMode {

    // Vision components
    private AprilTagProcessor aprilTag;
    private ColorBlobLocatorProcessor yellowLocator;
    private ColorBlobLocatorProcessor redLocator;
    private ColorBlobLocatorProcessor blueLocator;
    private ColorBlobLocatorProcessor purpleLocator;
    private ColorBlobLocatorProcessor tealLocator;
    private VisionPortal visionPortal;

    // Display toggles
    private boolean showAprilTags = true;
    private boolean showColorBlobs = true;
    private boolean lastA = false;
    private boolean lastB = false;
    private boolean lastX = false;

    // Color mode cycling (now includes Purple and Teal with tuned ranges)
    private enum ColorMode { YELLOW, RED, BLUE, PURPLE, TEAL }
    private ColorMode currentColorMode = ColorMode.YELLOW;

    // Camera settings (from old robot)
    private static final int CAMERA_WIDTH = 640;
    private static final int CAMERA_HEIGHT = 480;

    // ============================================================
    // CUSTOM COLOR RANGES - Retuned for better detection
    // Format: Scalar(Hue, Saturation, Value)
    // OpenCV HSV: Hue 0-180 | Saturation 0-255 | Value 0-255
    // ============================================================
    
    // PURPLE ARTIFACT - Narrowed hue, higher saturation to reject skin/shadows
    // Purple/Violet sits around 130-160 in OpenCV hue
    private static final ColorRange PURPLE_RANGE = new ColorRange(
            ColorSpace.HSV,
            new Scalar(125, 80, 60),    // Min: tighter hue, higher sat to reject skin
            new Scalar(160, 255, 255)   // Max: narrower range focused on violet
    );
    
    // TEAL/CYAN ARTIFACT - The "green" ball is actually more cyan/teal
    // Cyan sits around 80-100 in OpenCV hue (between green and blue)
    private static final ColorRange TEAL_RANGE = new ColorRange(
            ColorSpace.HSV,
            new Scalar(75, 150, 80),    // Min: shifted to cyan, high sat to reject gray
            new Scalar(105, 255, 255)   // Max: covers cyan/teal range
    );

    // Minimum blob area to filter out noise (in pixels)
    private static final double MIN_BLOB_AREA = 2500;

    @Override
    public void runOpMode() {
        // Initialize vision system
        initVision();

        telemetry.addLine("=== CAMERA TEST ===");
        telemetry.addData("Status", "Initialized");
        telemetry.addData("Camera", "Webcam 1 @ 640x480");
        telemetry.addLine();
        telemetry.addLine("Controls:");
        telemetry.addLine("  DPAD UP/DOWN: Stream On/Off");
        telemetry.addLine("  A: Toggle AprilTag info");
        telemetry.addLine("  B: Toggle Color info");
        telemetry.addLine("  X: Cycle color mode");
        telemetry.addLine();
        telemetry.addData(">", "Press START to begin");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            // Handle button toggles (with edge detection)
            handleButtons();

            // Camera stream controls
            handleStreamControl();

            // Build telemetry
            telemetry.addLine("=== CAMERA TEST ===");
            telemetry.addData("Stream", visionPortal.getCameraState());
            telemetry.addData("FPS", "%.1f", visionPortal.getFps());
            telemetry.addLine();

            // Display AprilTag data
            if (showAprilTags) {
                displayAprilTags();
            } else {
                telemetry.addLine("[AprilTags: HIDDEN - Press A]");
            }

            // Display color blob data
            if (showColorBlobs) {
                displayColorBlobs();
            } else {
                telemetry.addLine("[Colors: HIDDEN - Press B]");
            }

            // Control legend
            telemetry.addLine();
            telemetry.addLine("─── Controls ───");
            telemetry.addLine("DPAD: Stream | A: Tags | B: Color | X: Mode");

            telemetry.update();
            sleep(50);
        }

        visionPortal.close();
    }

    /**
     * Initialize AprilTag and color blob processors
     */
    private void initVision() {
        // Create AprilTag processor using calibration from CameraConstants
        aprilTag = new AprilTagProcessor.Builder()
                .setDrawAxes(true)
                .setDrawCubeProjection(true)
                .setDrawTagOutline(true)
                .setTagFamily(AprilTagProcessor.TagFamily.TAG_36h11)
                .setOutputUnits(DistanceUnit.CM, AngleUnit.DEGREES)
                .setLensIntrinsics(
                        CameraConstants.CAMERA_FX,
                        CameraConstants.CAMERA_FY,
                        CameraConstants.CAMERA_CX,
                        CameraConstants.CAMERA_CY)
                .build();

        // Create color blob locators for each color
        yellowLocator = new ColorBlobLocatorProcessor.Builder()
                .setTargetColorRange(ColorRange.YELLOW)
                .setContourMode(ColorBlobLocatorProcessor.ContourMode.EXTERNAL_ONLY)
                .setRoi(ImageRegion.entireFrame())
                .setDrawContours(true)
                .setBlurSize(5)
                .build();

        redLocator = new ColorBlobLocatorProcessor.Builder()
                .setTargetColorRange(ColorRange.RED)
                .setContourMode(ColorBlobLocatorProcessor.ContourMode.EXTERNAL_ONLY)
                .setRoi(ImageRegion.entireFrame())
                .setDrawContours(true)
                .setBlurSize(5)
                .build();

        blueLocator = new ColorBlobLocatorProcessor.Builder()
                .setTargetColorRange(ColorRange.BLUE)
                .setContourMode(ColorBlobLocatorProcessor.ContourMode.EXTERNAL_ONLY)
                .setRoi(ImageRegion.entireFrame())
                .setDrawContours(true)
                .setBlurSize(5)
                .build();

        // PURPLE - Custom tuned (narrower range, higher saturation)
        purpleLocator = new ColorBlobLocatorProcessor.Builder()
                .setTargetColorRange(PURPLE_RANGE)
                .setContourMode(ColorBlobLocatorProcessor.ContourMode.EXTERNAL_ONLY)
                .setRoi(ImageRegion.entireFrame())
                .setDrawContours(true)
                .setBlurSize(5)
                .build();

        // TEAL/CYAN - For the cyan-colored ball (shifted from green)
        tealLocator = new ColorBlobLocatorProcessor.Builder()
                .setTargetColorRange(TEAL_RANGE)
                .setContourMode(ColorBlobLocatorProcessor.ContourMode.EXTERNAL_ONLY)
                .setRoi(ImageRegion.entireFrame())
                .setDrawContours(true)
                .setBlurSize(5)
                .build();

        // Build vision portal
        VisionPortal.Builder builder = new VisionPortal.Builder();
        builder.setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"));
        builder.setCameraResolution(new Size(CAMERA_WIDTH, CAMERA_HEIGHT));
        builder.enableLiveView(true);
        builder.setStreamFormat(VisionPortal.StreamFormat.MJPEG);
        builder.addProcessor(aprilTag);
        builder.addProcessor(yellowLocator);
        builder.addProcessor(redLocator);
        builder.addProcessor(blueLocator);
        builder.addProcessor(purpleLocator);
        builder.addProcessor(tealLocator);

        visionPortal = builder.build();
    }

    /**
     * Handle button presses with edge detection
     */
    private void handleButtons() {
        // A button - toggle AprilTag display
        if (gamepad1.a && !lastA) {
            showAprilTags = !showAprilTags;
        }
        lastA = gamepad1.a;

        // B button - toggle color display
        if (gamepad1.b && !lastB) {
            showColorBlobs = !showColorBlobs;
        }
        lastB = gamepad1.b;

        // X button - cycle color mode
        if (gamepad1.x && !lastX) {
            cycleColorMode();
        }
        lastX = gamepad1.x;
    }

    /**
     * Handle camera stream controls
     */b
    private void handleStreamControl() {
        if (gamepad1.dpad_down) {
            visionPortal.stopStreaming();
        } else if (gamepad1.dpad_up) {
            visionPortal.resumeStreaming();
        }
    }

    /**
     * Cycle through color detection modes
     */
    private void cycleColorMode() {
        switch (currentColorMode) {
            case YELLOW:
                currentColorMode = ColorMode.RED;
                break;
            case RED:
                currentColorMode = ColorMode.BLUE;
                break;
            case BLUE:
                currentColorMode = ColorMode.PURPLE;
                break;
            case PURPLE:
                currentColorMode = ColorMode.TEAL;
                break;
            case TEAL:
                currentColorMode = ColorMode.YELLOW;
                break;
        }
    }

    /**
     * Display AprilTag detection info
     */
    private void displayAprilTags() {
        List<AprilTagDetection> detections = aprilTag.getDetections();

        telemetry.addLine("─── AprilTags ───");
        telemetry.addData("Detected", detections.size());

        for (AprilTagDetection detection : detections) {
            if (detection.metadata != null) {
                telemetry.addLine(String.format("  ID %d: %s", detection.id, detection.metadata.name));
                telemetry.addLine(String.format("    XYZ: %.1f, %.1f, %.1f cm",
                        detection.ftcPose.x, detection.ftcPose.y, detection.ftcPose.z));
                telemetry.addLine(String.format("    Range: %.1f cm  Bearing: %.1f°",
                        detection.ftcPose.range, detection.ftcPose.bearing));
            } else {
                telemetry.addLine(String.format("  ID %d: Unknown", detection.id));
                telemetry.addLine(String.format("    Center: (%.0f, %.0f) px",
                        detection.center.x, detection.center.y));
            }
        }
        telemetry.addLine();
    }

    /**
     * Display color blob detection info
     */
    private void displayColorBlobs() {
        // Get blobs from the currently selected color locator
        List<ColorBlobLocatorProcessor.Blob> blobs;
        switch (currentColorMode) {
            case RED:
                blobs = redLocator.getBlobs();
                break;
            case BLUE:
                blobs = blueLocator.getBlobs();
                break;
            case PURPLE:
                blobs = purpleLocator.getBlobs();
                break;
            case TEAL:
                blobs = tealLocator.getBlobs();
                break;
            case YELLOW:
            default:
                blobs = yellowLocator.getBlobs();
                break;
        }

        telemetry.addLine("─── Color Blobs ───");
        telemetry.addData("Mode", currentColorMode.toString() + " [X to change]");
        
        // Filter blobs by minimum area to remove noise
        int totalBlobs = blobs.size();
        int filteredCount = 0;

        // Show top 3 largest blobs that pass the area filter
        int count = 0;
        for (ColorBlobLocatorProcessor.Blob blob : blobs) {
            // Skip small noise blobs
            if (blob.getContourArea() < MIN_BLOB_AREA) {
                continue;
            }
            filteredCount++;
            
            if (count >= 3) continue; // Only display first 3

            RotatedRect box = blob.getBoxFit();
            telemetry.addLine(String.format("  Blob %d: Area=%.0f", count + 1, blob.getContourArea()));
            telemetry.addLine(String.format("    Center: (%.0f, %.0f) px", box.center.x, box.center.y));
            telemetry.addLine(String.format("    Size: %.0f x %.0f px", box.size.width, box.size.height));
            
            // Position hint
            String posHint;
            if (box.center.x < CAMERA_WIDTH / 3.0) {
                posHint = "LEFT";
            } else if (box.center.x > CAMERA_WIDTH * 2.0 / 3.0) {
                posHint = "RIGHT";
            } else {
                posHint = "CENTER";
            }
            telemetry.addLine(String.format("    Position: %s", posHint));
            
            count++;
        }
        
        telemetry.addData("Valid Blobs", filteredCount + " (filtered from " + totalBlobs + ")");
        telemetry.addLine();
    }
}
