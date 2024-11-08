package org.firstinspires.ftc.teamcode.rr.drive;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.drive.MecanumDrive;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.arcrobotics.ftclib.geometry.Rotation2d;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.jetbrains.annotations.NotNull;

import java.util.Collections;
import java.util.List;

import static org.firstinspires.ftc.teamcode.rr.drive.DriveConstants.*;


/*
 * Simple mecanum drive hardware implementation for REV hardware.
 */
@Config
public class SampleMecanumDrive extends MecanumDrive {

    private IMU imu;

    public SampleMecanumDrive(HardwareMap hardwareMap) {
        super(kV, kA, kStatic, TRACK_WIDTH, TRACK_WIDTH, 1);

        imu = hardwareMap.get(IMU.class, "imu");
        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                DriveConstants.LOGO_FACING_DIR, DriveConstants.USB_FACING_DIR));
        imu.initialize(parameters);

        setLocalizer(new TwoWheelTrackingLocalizer(hardwareMap, this));
    }

    public com.arcrobotics.ftclib.geometry.Pose2d getPose() {
        Pose2d pose = this.getPoseEstimate();
        return new com.arcrobotics.ftclib.geometry.Pose2d(pose.getX(), -pose.getY(), new Rotation2d(-pose.getHeading()));
    }

    public void update() {
        updatePoseEstimate();
    }

    @Override
    public double getRawExternalHeading() {
        return -imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);
    }

    @Override
    public Double getExternalHeadingVelocity() {
        return -((double) imu.getRobotAngularVelocity(AngleUnit.RADIANS).zRotationRate);
    }

    @Override
    public void setMotorPowers(double v, double v1, double v2, double v3) {
        // Unused
    }

    @Override
    public @NotNull List<Double> getWheelPositions() {
        return Collections.emptyList();
    }
}