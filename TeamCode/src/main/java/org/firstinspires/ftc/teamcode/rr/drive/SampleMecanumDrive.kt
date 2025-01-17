package org.firstinspires.ftc.teamcode.rr.drive

import com.acmerobotics.dashboard.config.Config
import com.acmerobotics.roadrunner.drive.MecanumDrive
import com.arcrobotics.ftclib.geometry.Pose2d
import com.arcrobotics.ftclib.geometry.Rotation2d
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot
import com.qualcomm.robotcore.hardware.HardwareMap
import com.qualcomm.robotcore.hardware.IMU
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit
import org.firstinspires.ftc.teamcode.rr.drive.DriveConstants.TRACK_WIDTH
import org.firstinspires.ftc.teamcode.rr.drive.DriveConstants.kA
import org.firstinspires.ftc.teamcode.rr.drive.DriveConstants.kStatic
import org.firstinspires.ftc.teamcode.rr.drive.DriveConstants.kV

/*
 * Simple mecanum drive hardware implementation for REV hardware.
 */
@Config
class SampleMecanumDrive(
    hardwareMap: HardwareMap,
) : MecanumDrive(kV, kA, kStatic, TRACK_WIDTH, TRACK_WIDTH, 1.0) {
    private val imu: IMU = hardwareMap.get(IMU::class.java, "imu")

    public override val rawExternalHeading: Double
        get() = -imu.robotYawPitchRollAngles.getYaw(AngleUnit.RADIANS)

    init {
        val parameters =
            IMU.Parameters(
                RevHubOrientationOnRobot(
                    DriveConstants.LOGO_FACING_DIR,
                    DriveConstants.USB_FACING_DIR,
                ),
            )
        imu.initialize(parameters)

        localizer = TwoWheelTrackingLocalizer(hardwareMap, this)
    }

    fun getPose(): Pose2d {
        val pose = this.poseEstimate
        return Pose2d(pose.x, -pose.y, Rotation2d(-pose.heading))
    }

    fun update() {
        updatePoseEstimate()
    }

    override fun getExternalHeadingVelocity(): Double = -imu.getRobotAngularVelocity(AngleUnit.RADIANS).zRotationRate.toDouble()

    override fun setMotorPowers(
        v: Double,
        v1: Double,
        v2: Double,
        v3: Double,
    ) {
        // Unused
    }

    override fun getWheelPositions(): List<Double> = emptyList()
}
