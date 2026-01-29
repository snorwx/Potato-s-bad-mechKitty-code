package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * AutoForwardAndSpin
 * Drives forward for 2 seconds, stops, then spins right for 5 seconds.
 * Works when you press INIT then ▶️ (Triangle) START.
 */
@Autonomous(name = "Auto: Forward and Spin", group = "Robot")
public class auto extends LinearOpMode {

    private RobotHardware robot = new RobotHardware();
    private ElapsedTime runtime = new ElapsedTime();

    static final double DRIVE_SPEED = 0.5;
    static final double TURN_SPEED = 0.5;

    @Override
    public void runOpMode() {

        telemetry.addLine("🔧 Initializing hardware...");
        telemetry.update();

        try {
            robot.init(hardwareMap);
            telemetry.addLine("✅ Robot hardware initialized.");
        } catch (Exception e) {
            telemetry.addLine("❌ Hardware init failed: " + e.getMessage());
            telemetry.update();
            return;
        }

        // Basic hardware check
        if (robot.frontLeft == null || robot.frontRight == null ||
                robot.backLeft == null || robot.backRight == null) {
            telemetry.addLine("⚠️ ERROR: One or more drive motors not mapped!");
            telemetry.update();
            sleep(3000);
            return;
        }

        telemetry.addLine("✅ Ready to start.");
        telemetry.addLine("Press ▶️ (Triangle) to begin autonomous.");
        telemetry.update();

        waitForStart();  // Wait until Triangle is pressed

        if (opModeIsActive()) {

            telemetry.addLine("🚗 Step 1: Drive Forward (2 seconds)");
            telemetry.update();

            // Drive forward
            setDrivePower(DRIVE_SPEED, DRIVE_SPEED, DRIVE_SPEED, DRIVE_SPEED);
            runtime.reset();

            while (opModeIsActive() && runtime.seconds() < 1.0) {
                telemetry.addData("Driving...", "%.1f s", runtime.seconds());
                telemetry.update();
                idle();
            }

            stopAllMotors();
            sleep(500);

            telemetry.addLine("🔄 Step 2: Spin Right (5 seconds)");
            telemetry.update();

            // Spin right — left side forward, right side backward
            setDrivePower(TURN_SPEED, -TURN_SPEED, TURN_SPEED, -TURN_SPEED);
            runtime.reset();

            while (opModeIsActive() && runtime.seconds() < 5.0) {
                telemetry.addData("Spinning...", "%.1f s", runtime.seconds());
                telemetry.update();
                idle();
            }

            stopAllMotors();

            telemetry.addLine("🏁 Autonomous Complete!");
            telemetry.update();
            sleep(1000);
        }
    }

    /** Helper to set drive motor powers */
    private void setDrivePower(double fl, double fr, double bl, double br) {
        robot.frontLeft.setPower(fl);
        robot.frontRight.setPower(fr);
        robot.backLeft.setPower(bl);
        robot.backRight.setPower(br);
    }

    /** Stop all drivetrain motors */
    private void stopAllMotors() {
        setDrivePower(0, 0, 0, 0);
    }
}
