package org.firstinspires.ftc.teamcode.common.subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.geometry.Pose2d;
import com.arcrobotics.ftclib.geometry.Rotation2d;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.arcrobotics.ftclib.kinematics.HolonomicOdometry;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.VoltageSensor;

import java.util.List;

@Config
public class DriveSubsystem extends SubsystemBase {
    private double cacheThreshold = 0.1; // -1 to disable
    private double[] prevPower = new double[]{0.0, 0.0, 0.0, 0.0};
    public MotorEx leftFront, leftRear, rightRear, rightFront;

    public IMU imu;
    private VoltageSensor batteryVoltageSensor;

    public static double TRACK_WIDTH = -12.375;
    public static double CENTER_WHEEL_OFFSET = 0.5; // distance between center of rotation of the robot and the center odometer
    public static double WHEEL_DIAMETER = 1.45;
    public static double TICKS_PER_REV = 8192;
    public static double DISTANCE_PER_PULSE = Math.PI * WHEEL_DIAMETER / TICKS_PER_REV;
    public static boolean BREAK = false;

    public static double startingX = -60.0;
    public static double startingY = -60.0;
    public static double startingH = 0.0;

    public Motor.Encoder leftOdom, rightOdom, centerOdom;
    public HolonomicOdometry odometry;

    public DriveSubsystem(HardwareMap hardwareMap, double cacheThreshold) {
        this.cacheThreshold = cacheThreshold;
        DISTANCE_PER_PULSE = Math.PI * WHEEL_DIAMETER / TICKS_PER_REV;

        imu = hardwareMap.get(IMU.class, "imu");

        leftFront = new MotorEx(hardwareMap, "frontLeft");
        leftFront.setInverted(true);

        leftRear = new MotorEx(hardwareMap, "backLeft");
        leftRear.setInverted(true);

        rightRear = new MotorEx(hardwareMap, "backRight");
        rightRear.setInverted(false);

        rightFront = new MotorEx(hardwareMap, "frontRight");
        rightFront.setInverted(false);

        Motor.ZeroPowerBehavior stop = BREAK ? Motor.ZeroPowerBehavior.BRAKE : Motor.ZeroPowerBehavior.FLOAT;

        rightRear.setZeroPowerBehavior(stop);
        rightFront.setZeroPowerBehavior(stop);
        leftRear.setZeroPowerBehavior(stop);
        leftFront.setZeroPowerBehavior(stop);

        imu = hardwareMap.get(IMU.class, "imu");
        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.LEFT,
                RevHubOrientationOnRobot.UsbFacingDirection.UP
        ));
        imu.initialize(parameters);
        imu.resetYaw();

        leftOdom = rightRear.encoder.setDistancePerPulse(DISTANCE_PER_PULSE);
        rightOdom = leftFront.encoder.setDistancePerPulse(DISTANCE_PER_PULSE);
        centerOdom = leftRear.encoder.setDistancePerPulse(DISTANCE_PER_PULSE);

        leftOdom.setDirection(Motor.Direction.FORWARD);
        rightOdom.setDirection(Motor.Direction.REVERSE);
        centerOdom.setDirection(Motor.Direction.REVERSE);

        leftOdom.reset();
        rightOdom.reset();
        centerOdom.reset();

        odometry = new HolonomicOdometry(
                leftOdom::getDistance,
                rightOdom::getDistance,
                centerOdom::getDistance,
                TRACK_WIDTH, CENTER_WHEEL_OFFSET
        );

        // change to reflect starting field position
        odometry.updatePose(new com.arcrobotics.ftclib.geometry.Pose2d(startingX, startingY, new Rotation2d(Math.toRadians(startingH))));

        List<LynxModule> allHubs = hardwareMap.getAll(LynxModule.class);

        for (LynxModule hub : allHubs) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
        }
    }

    public void robotCentric(double power, double strafe, double turn) {
        robotCentric(power, strafe, turn, 1, 1);
    }

    public void robotCentric(double power, double strafe, double turn, double multiF, double multiH) {
        double denominator = Math.max(Math.abs(power * 1 / multiF) + Math.abs(strafe * 1 / multiF) + Math.abs(turn * 1 / multiH), Math.max(1, Math.max(1 / multiF, 1 / multiH)));
        double frontLeftPower = (power + strafe + turn) / denominator;
        double backLeftPower = (power - strafe + turn) / denominator;
        double frontRightPower = (power - strafe - turn) / denominator;
        double backRightPower = (power + strafe - turn) / denominator;

        boolean caching = cacheThreshold == -1;
        if (caching) {
            double frontLeftDiff = Math.abs(prevPower[0] - frontLeftPower);
            double backLeftDiff = Math.abs(prevPower[1] - backLeftPower);
            double frontRightDiff = Math.abs(prevPower[2] - frontRightPower);
            double backRightDiff = Math.abs(prevPower[3] - backRightPower);
            if (frontLeftDiff > cacheThreshold) {
                prevPower[0] = frontLeftPower;
                leftFront.set(frontLeftPower);
            }
            if (backLeftDiff > cacheThreshold) {
                prevPower[1] = backLeftPower;
                leftRear.set(backLeftPower);
            }
            if (frontRightDiff > cacheThreshold) {
                prevPower[2] = frontRightPower;
                rightFront.set(frontRightPower);
            }
            if (backRightDiff > cacheThreshold) {
                prevPower[3] = backRightPower;
                rightRear.set(backRightPower);
            }
        } else {
            leftFront.set(frontLeftPower);
            leftRear.set(backLeftPower);
            rightFront.set(frontRightPower);
            rightRear.set(backRightPower);
        }
    }

    public void fieldCentric(double x, double y, double rx, double heading) {
        // Rotate the movement direction counter to the bot's rotation
        double rotX = x * Math.cos(-heading) - y * Math.sin(-heading);
        double rotY = x * Math.sin(-heading) + y * Math.cos(-heading);

        rotX = rotX * 1.1;
        robotCentric(rotY, rotX, rx);
    }

    public void updatePos() {
        odometry.updatePose();
    }

    public Pose2d getPos() {
        return odometry.getPose();
    }
}
