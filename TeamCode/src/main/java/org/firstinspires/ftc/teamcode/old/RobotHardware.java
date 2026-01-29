package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

public class RobotHardware {
    // Drivetrain motors
    public DcMotor frontLeft, frontRight, backLeft, backRight;
    
    // IMU for odometry/heading
    public IMU imu;

    // Hardware name constants
    private static final String FRONT_LEFT = "frontLeft";
    private static final String FRONT_RIGHT = "frontRight";
    private static final String BACK_LEFT = "backLeft";
    private static final String BACK_RIGHT = "backRight";
    private static final String IMU_NAME = "imu";

    public void init(HardwareMap hardwareMap) {
        // Drivetrain
        frontLeft  = hardwareMap.get(DcMotor.class, FRONT_LEFT);
        frontRight = hardwareMap.get(DcMotor.class, FRONT_RIGHT);
        backLeft   = hardwareMap.get(DcMotor.class, BACK_LEFT);
        backRight  = hardwareMap.get(DcMotor.class, BACK_RIGHT);

        // Motor directions
        frontLeft.setDirection(DcMotor.Direction.REVERSE);
        backLeft.setDirection(DcMotor.Direction.REVERSE);

        // Setup all drive motors
        DcMotor[] driveMotors = {frontLeft, frontRight, backLeft, backRight};
        for (DcMotor m : driveMotors) {
            m.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            m.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            m.setPower(0);
        }

        // IMU for heading/odometry
        imu = hardwareMap.get(IMU.class, IMU_NAME);
        IMU.Parameters parameters = new IMU.Parameters(
                new RevHubOrientationOnRobot(
                        RevHubOrientationOnRobot.LogoFacingDirection.UP,
                        RevHubOrientationOnRobot.UsbFacingDirection.FORWARD
                )
        );
        imu.initialize(parameters);
    }

    /**
     * Standard mecanum drive power distribution
     * @param axial Forward/backward (-1 to 1)
     * @param lateral Strafe left/right (-1 to 1)
     * @param yaw Rotation (-1 to 1)
     */
    public void drive(double axial, double lateral, double yaw) {
        double fl = (axial + lateral + yaw) * 0.95;
        double bl = (axial - lateral + yaw) * 0.95;
        double fr = (axial - lateral - yaw);
        double br = (axial + lateral - yaw);

        double max = Math.max(1.0, Math.max(Math.abs(fl),
                Math.max(Math.abs(fr), Math.max(Math.abs(bl), Math.abs(br)))));

        frontLeft.setPower(fl / max);
        backLeft.setPower(bl / max);
        frontRight.setPower(fr / max);
        backRight.setPower(br / max);
    }

    /**
     * Stop all drive motors
     */
    public void stop() {
        frontLeft.setPower(0);
        frontRight.setPower(0);
        backLeft.setPower(0);
        backRight.setPower(0);
    }

    /**
     * Get robot heading from IMU
     * @return Heading in degrees
     */
    public double getHeading() {
        return imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
    }

    /**
     * Reset IMU heading to zero
     */
    public void resetHeading() {
        imu.resetYaw();
    }
}
