package org.firstinspires.ftc.teamcode;

import android.util.Size;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import org.firstinspires.ftc.vision.opencv.ColorBlobLocatorProcessor;
import org.firstinspires.ftc.vision.opencv.ColorRange;
import org.firstinspires.ftc.vision.opencv.ColorSpace;
import org.firstinspires.ftc.vision.opencv.ImageRegion;
import org.opencv.core.Scalar;

import java.util.List;

@TeleOp(name = "TeleOp Vision", group = "Linear Opmode")
public class TeleOpObeliskDetection extends LinearOpMode {

    private RobotHardware robot = new RobotHardware();
    private AprilTagProcessor aprilTag;
    private ColorBlobLocatorProcessor purpleLocator;
    private ColorBlobLocatorProcessor greenLocator;
    private VisionPortal visionPortal;

    // ============================================================
    // COLOR RANGES FOR FTC DECODE 2025 ARTIFACTS
    // Format: Scalar(Hue, Saturation, Value)
    // Hue: 0-180 in OpenCV | Saturation: 0-255 | Value: 0-255
    // ============================================================
    
    // PURPLE ARTIFACT - Wide range for varying lighting conditions
    private static final Scalar PURPLE_MIN = new Scalar(115, 50, 50);
    private static final Scalar PURPLE_MAX = new Scalar(175, 255, 255);
    
    // GREEN ARTIFACT - High saturation to reject gray/black items
    private static final Scalar GREEN_MIN = new Scalar(45, 130, 70);
    private static final Scalar GREEN_MAX = new Scalar(85, 255, 255);
    
    // ============================================================
    // SHAPE FILTERING PARAMETERS
    // ============================================================
    private static final int MIN_BLOB_AREA = 2500;
    private static final int MAX_BLOB_AREA = 150000;
    private static final double MAX_ASPECT_RATIO = 1.6;
    private static final double MIN_DENSITY = 0.40;
    private static final double MAX_DENSITY = 0.90;
    private static final double MIN_CIRCULARITY = 0.30;

    // Obelisk AprilTag IDs (11-16)
    private static final int[] RED_OBELISK_IDS = {11, 12, 13};
    private static final int[] BLUE_OBELISK_IDS = {14, 15, 16};
    private static final int[] ALL_OBELISK_IDS = {11, 12, 13, 14, 15, 16};
    
    // Detection tracking
    private int detectedObeliskID = -1;
    private double obeliskDistance = 0;
    private double obeliskBearing = 0;
    private String obeliskAlliance = "UNKNOWN";
    
    // Artifact Detection Tracking
    private boolean purpleDetected = false;
    private double purpleX = 0;
    private double purpleArea = 0;
    
    private boolean greenDetected = false;
    private double greenX = 0;
    private double greenArea = 0;
    
    // Diagnostics
    private int purpleBlobsTotal = 0;
    private int greenBlobsTotal = 0;
    private String purpleRejectReason = "";
    private String greenRejectReason = "";

    @Override
    public void runOpMode() {
        robot.init(hardwareMap);
        initVision();

        telemetry.addData("Status", "Initialized");
        telemetry.addData("Camera", "Ready");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            // === DRIVE CONTROL ===
            double axial   = -gamepad1.left_stick_y;
            double lateral =  gamepad1.left_stick_x;
            double yaw     =  gamepad1.right_stick_x;
            robot.drive(axial, lateral, yaw);

            // === VISION DETECTION ===
            detectObelisks();
            detectArtifacts();

            // === CAMERA STREAM CONTROL ===
            if (gamepad1.dpad_down) {
                visionPortal.stopStreaming();
            } else if (gamepad1.dpad_up) {
                visionPortal.resumeStreaming();
            }

            // === TELEMETRY ===
            telemetry.addLine("=== DRIVE ===");
            telemetry.addData("Heading", "%.1f°", robot.getHeading());
            telemetry.addData("FL | FR", "%.2f | %.2f", robot.frontLeft.getPower(), robot.frontRight.getPower());
            telemetry.addData("BL | BR", "%.2f | %.2f", robot.backLeft.getPower(), robot.backRight.getPower());
            
            telemetry.addLine("\n=== OBELISK DETECTION ===");
            if (detectedObeliskID != -1) {
                telemetry.addData("OBELISK", "ID %d (%s)", detectedObeliskID, obeliskAlliance);
                telemetry.addData("Distance", "%.1f in", obeliskDistance);
                telemetry.addData("Bearing", "%.1f°", obeliskBearing);
            } else {
                telemetry.addData("OBELISK", "Not Detected");
            }
            
            telemetry.addLine("\n=== ARTIFACT DETECTION ===");
            
            if (purpleDetected) {
                telemetry.addData("PURPLE", "FOUND (Area: %.0f)", purpleArea);
                telemetry.addData("  Position", "X: %.0f %s", purpleX, 
                    Math.abs(purpleX - 320) < 30 ? "[CENTERED]" : (purpleX < 320 ? "[LEFT]" : "[RIGHT]"));
            } else {
                telemetry.addData("PURPLE", "Not detected (%d blobs, %s)", 
                    purpleBlobsTotal, purpleRejectReason.isEmpty() ? "-" : purpleRejectReason);
            }
            
            if (greenDetected) {
                telemetry.addData("GREEN", "FOUND (Area: %.0f)", greenArea);
                telemetry.addData("  Position", "X: %.0f %s", greenX,
                    Math.abs(greenX - 320) < 30 ? "[CENTERED]" : (greenX < 320 ? "[LEFT]" : "[RIGHT]"));
            } else {
                telemetry.addData("GREEN", "Not detected (%d blobs, %s)", 
                    greenBlobsTotal, greenRejectReason.isEmpty() ? "-" : greenRejectReason);
            }
            
            telemetry.addLine("\n[DPAD UP/DOWN: Camera On/Off]");
            telemetry.update();
            
            idle();
        }
        
        visionPortal.close();
    }

    private void initVision() {
        aprilTag = new AprilTagProcessor.Builder()
                .setDrawAxes(true)
                .setDrawCubeProjection(true)
                .setDrawTagOutline(true)
                .build();

        purpleLocator = new ColorBlobLocatorProcessor.Builder()
                .setTargetColorRange(new ColorRange(ColorSpace.HSV, PURPLE_MIN, PURPLE_MAX))
                .setContourMode(ColorBlobLocatorProcessor.ContourMode.EXTERNAL_ONLY)
                .setRoi(ImageRegion.entireFrame())
                .setDrawContours(true)
                .setBlurSize(9)
                .build();
        
        greenLocator = new ColorBlobLocatorProcessor.Builder()
                .setTargetColorRange(new ColorRange(ColorSpace.HSV, GREEN_MIN, GREEN_MAX))
                .setContourMode(ColorBlobLocatorProcessor.ContourMode.EXTERNAL_ONLY)
                .setRoi(ImageRegion.entireFrame())
                .setDrawContours(true)
                .setBlurSize(9)
                .build();

        visionPortal = new VisionPortal.Builder()
                .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"))
                .setCameraResolution(new Size(640, 480))
                .enableLiveView(true)
                .addProcessor(aprilTag)
                .addProcessor(purpleLocator)
                .addProcessor(greenLocator)
                .build();
    }

    private boolean isValidArtifactShape(ColorBlobLocatorProcessor.Blob blob) {
        double boxWidth = blob.getBoxFit().size.width;
        double boxHeight = blob.getBoxFit().size.height;
        int contourArea = blob.getContourArea();
        
        if (boxWidth == 0 || boxHeight == 0) return false;
        if (contourArea < MIN_BLOB_AREA || contourArea > MAX_BLOB_AREA) return false;
        
        double aspectRatio = Math.max(boxWidth / boxHeight, boxHeight / boxWidth);
        if (aspectRatio > MAX_ASPECT_RATIO) return false;
        
        double density = contourArea / (boxWidth * boxHeight);
        if (density < MIN_DENSITY || density > MAX_DENSITY) return false;
        
        double perimeter = blob.getContourPoints().length * 2.0;
        if (perimeter > 0) {
            double circularity = (4.0 * Math.PI * contourArea) / (perimeter * perimeter);
            if (circularity < MIN_CIRCULARITY) return false;
        }
        
        return true;
    }

    private void detectArtifacts() {
        // Purple detection
        List<ColorBlobLocatorProcessor.Blob> purpleBlobs = purpleLocator.getBlobs();
        purpleBlobsTotal = purpleBlobs.size();
        purpleDetected = false;
        purpleX = 0;
        purpleArea = 0;
        purpleRejectReason = purpleBlobsTotal == 0 ? "No color" : "";
        
        double bestPurpleScore = 0;
        for (ColorBlobLocatorProcessor.Blob blob : purpleBlobs) {
            int area = blob.getContourArea();
            if (area < MIN_BLOB_AREA) { purpleRejectReason = "Too small"; continue; }
            if (area > MAX_BLOB_AREA) { purpleRejectReason = "Too large"; continue; }
            if (!isValidArtifactShape(blob)) { purpleRejectReason = "Bad shape"; continue; }
            
            double aspectRatio = Math.max(blob.getBoxFit().size.width / blob.getBoxFit().size.height, 
                                          blob.getBoxFit().size.height / blob.getBoxFit().size.width);
            double score = area * (2.0 - aspectRatio);
            
            if (score > bestPurpleScore) {
                bestPurpleScore = score;
                purpleArea = area;
                purpleX = blob.getBoxFit().center.x;
                purpleDetected = true;
                purpleRejectReason = "";
            }
        }
        
        // Green detection
        List<ColorBlobLocatorProcessor.Blob> greenBlobs = greenLocator.getBlobs();
        greenBlobsTotal = greenBlobs.size();
        greenDetected = false;
        greenX = 0;
        greenArea = 0;
        greenRejectReason = greenBlobsTotal == 0 ? "No color" : "";
        
        double bestGreenScore = 0;
        for (ColorBlobLocatorProcessor.Blob blob : greenBlobs) {
            int area = blob.getContourArea();
            if (area < MIN_BLOB_AREA) { greenRejectReason = "Too small"; continue; }
            if (area > MAX_BLOB_AREA) { greenRejectReason = "Too large"; continue; }
            if (!isValidArtifactShape(blob)) { greenRejectReason = "Bad shape"; continue; }
            
            double aspectRatio = Math.max(blob.getBoxFit().size.width / blob.getBoxFit().size.height,
                                          blob.getBoxFit().size.height / blob.getBoxFit().size.width);
            double score = area * (2.0 - aspectRatio);
            
            if (score > bestGreenScore) {
                bestGreenScore = score;
                greenArea = area;
                greenX = blob.getBoxFit().center.x;
                greenDetected = true;
                greenRejectReason = "";
            }
        }
    }

    private void detectObelisks() {
        List<AprilTagDetection> detections = aprilTag.getDetections();
        detectedObeliskID = -1;
        obeliskDistance = 0;
        obeliskBearing = 0;
        obeliskAlliance = "UNKNOWN";

        for (AprilTagDetection detection : detections) {
            if (isObeliskTag(detection.id)) {
                detectedObeliskID = detection.id;
                obeliskAlliance = getObeliskAlliance(detection.id);
                if (detection.ftcPose != null) {
                    obeliskDistance = detection.ftcPose.range;
                    obeliskBearing = detection.ftcPose.bearing;
                }
                break;
            }
        }
    }

    private boolean isObeliskTag(int tagId) {
        for (int id : ALL_OBELISK_IDS) if (tagId == id) return true;
        return false;
    }
    
    private String getObeliskAlliance(int tagId) {
        for (int id : RED_OBELISK_IDS) if (tagId == id) return "RED";
        for (int id : BLUE_OBELISK_IDS) if (tagId == id) return "BLUE";
        return "UNKNOWN";
    }
}
