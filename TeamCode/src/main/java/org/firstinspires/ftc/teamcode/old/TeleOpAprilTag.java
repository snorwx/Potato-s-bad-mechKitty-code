package org.firstinspires.ftc.teamcode;

import android.util.Size;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;

/**
 * Simple TeleOp mode for testing AprilTag detection
 * Displays detected AprilTag information on telemetry
 * Use DPAD_UP to resume streaming, DPAD_DOWN to stop streaming
 */
@TeleOp(name = "AprilTag Test", group = "Test")
public class TeleOpAprilTag extends LinearOpMode {

    private AprilTagProcessor aprilTag;
    private VisionPortal visionPortal;

    @Override
    public void runOpMode() {
        // Initialize AprilTag detection
        initAprilTag();

        // Display initialization status
        telemetry.addData("Status", "Initialized");
        telemetry.addData("Camera", "Ready");
        telemetry.addData(">", "Press START to begin AprilTag detection");
        telemetry.update();

        waitForStart();

        if (opModeIsActive()) {
            while (opModeIsActive()) {
                // Display AprilTag detection data
                telemetryAprilTag();

                // Camera stream controls
                if (gamepad1.dpad_down) {
                    visionPortal.stopStreaming();
                    telemetry.addData("Stream", "STOPPED");
                } else if (gamepad1.dpad_up) {
                    visionPortal.resumeStreaming();
                    telemetry.addData("Stream", "ACTIVE");
                }

                // Update telemetry
                telemetry.update();

                // Save CPU resources
                sleep(20);
            }
        }

        // Close the vision portal when done
        visionPortal.close();
    }

    /**
     * Initialize the AprilTag processor and vision portal
     */
    private void initAprilTag() {
        // Create the AprilTag processor with default settings
        aprilTag = new AprilTagProcessor.Builder()
                .setDrawAxes(true)
                .setDrawCubeProjection(true)
                .setDrawTagOutline(true)
                .build();

        // Create the vision portal
        VisionPortal.Builder builder = new VisionPortal.Builder();

        // Set the camera to webcam
        builder.setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"));

        // Optional: Set camera resolution
        builder.setCameraResolution(new Size(640, 480));

        // Enable live view
        builder.enableLiveView(true);

        // Add the AprilTag processor
        builder.addProcessor(aprilTag);

        // Build the vision portal
        visionPortal = builder.build();
    }

    /**
     * Display AprilTag detection information on telemetry
     */
    private void telemetryAprilTag() {
        List<AprilTagDetection> currentDetections = aprilTag.getDetections();

        telemetry.addData("# AprilTags Detected", currentDetections.size());
        telemetry.addLine();

        // Display information for each detected AprilTag
        for (AprilTagDetection detection : currentDetections) {
            if (detection.metadata != null) {
                // Display detailed information for known tags
                telemetry.addLine(String.format("==== TAG ID %d: %s ====",
                        detection.id, detection.metadata.name));
                telemetry.addLine(String.format("Position (X, Y, Z): %.1f, %.1f, %.1f inches",
                        detection.ftcPose.x, detection.ftcPose.y, detection.ftcPose.z));
                telemetry.addLine(String.format("Rotation (P, R, Y): %.1f°, %.1f°, %.1f°",
                        detection.ftcPose.pitch, detection.ftcPose.roll, detection.ftcPose.yaw));
                telemetry.addLine(String.format("Range: %.1f inches",
                        detection.ftcPose.range));
                telemetry.addLine(String.format("Bearing: %.1f°",
                        detection.ftcPose.bearing));
                telemetry.addLine(String.format("Elevation: %.1f°",
                        detection.ftcPose.elevation));
            } else {
                // Display basic information for unknown tags
                telemetry.addLine(String.format("==== TAG ID %d: Unknown ====",
                        detection.id));
                telemetry.addLine(String.format("Center: (%.0f, %.0f) pixels",
                        detection.center.x, detection.center.y));
            }
            telemetry.addLine();
        }

        // Display legend
        telemetry.addLine("───────────────────────");
        telemetry.addLine("Controls:");
        telemetry.addLine("  DPAD_UP: Resume stream");
        telemetry.addLine("  DPAD_DOWN: Stop stream");
        telemetry.addLine();
        telemetry.addLine("Position Info:");
        telemetry.addLine("  X = Right(+) / Left(-)");
        telemetry.addLine("  Y = Forward(+) / Back(-)");
        telemetry.addLine("  Z = Up(+) / Down(-)");
    }
}
