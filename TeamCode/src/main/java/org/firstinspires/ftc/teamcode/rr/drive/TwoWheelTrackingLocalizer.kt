package org.firstinspires.ftc.teamcode.rr.drive

import com.acmerobotics.dashboard.config.Config
import com.acmerobotics.roadrunner.geometry.Pose2d
import com.acmerobotics.roadrunner.localization.TwoTrackingWheelLocalizer
import com.qualcomm.robotcore.hardware.DcMotorEx
import com.qualcomm.robotcore.hardware.HardwareMap
import org.firstinspires.ftc.teamcode.rr.util.Encoder

/*
 * Sample tracking wheel localizer implementation assuming the standard configuration:
 *
 *    ^
 *    |
 *    | ( x direction)
 *    |
 *    v
 *    <----( y direction )---->

 *        (forward)
 *    /--------------\
 *    |     ____     |
 *    |     ----     |    <- Perpendicular Wheel
 *    |           || |
 *    |           || |    <- Parallel Wheel
 *    |              |
 *    |              |
 *    \--------------/
 *
 */
@Config
class TwoWheelTrackingLocalizer(
    hardwareMap: HardwareMap,
    var drive: SampleMecanumDrive,
) : TwoTrackingWheelLocalizer(
        listOf(
            Pose2d(PARALLEL_X, PARALLEL_Y, 0.0),
            Pose2d(PERPENDICULAR_X, PERPENDICULAR_Y, Math.toRadians(90.0)),
        ),
    ) {
    // Parallel/Perpendicular to the forward axis
    // Parallel wheel is parallel to the forward axis
    // Perpendicular is perpendicular to the forward axis
    var parallelEncoder: Encoder =
        Encoder(hardwareMap.get(DcMotorEx::class.java, "para")).apply {
            direction = Encoder.Direction.FORWARD
        }
    var perpendicularEncoder: Encoder =
        Encoder(hardwareMap.get(DcMotorEx::class.java, "frontLeft")).apply {
            direction = Encoder.Direction.FORWARD
        }

    override fun getHeading(): Double = -drive.rawExternalHeading

    override fun getHeadingVelocity(): Double {
//        return drive.getExternalHeadingVelocity()
        val velo = drive.getExternalHeadingVelocity()
        return -velo
    }

    override fun getWheelPositions(): List<Double> =
        listOf(
            encoderTicksToInches(parallelEncoder.getCurrentPosition().toDouble()) * X_MULTIPLIER,
            encoderTicksToInches(perpendicularEncoder.getCurrentPosition().toDouble()) * Y_MULTIPLIER,
        )

    override fun getWheelVelocities(): List<Double> =
        listOf(
            encoderTicksToInches(parallelEncoder.getCorrectedVelocity()) * X_MULTIPLIER,
            encoderTicksToInches(perpendicularEncoder.getCorrectedVelocity()) * Y_MULTIPLIER,
        )

    companion object {
        @JvmField
        var TICKS_PER_REV: Double = 8192.0

        @JvmField
        var WHEEL_RADIUS: Double = 35 / 25.4 // in

        @JvmField
        var GEAR_RATIO: Double = 1.0 // output (wheel) speed / input (encoder) speed

        @JvmField
        var PARALLEL_X: Double = 2.5 // X is the up and down direction

        @JvmField
        var PARALLEL_Y: Double = 6.0 // Y is the strafe direction

        @JvmField
        var PERPENDICULAR_X: Double = -0.875

        @JvmField
        var PERPENDICULAR_Y: Double = 0.375

        @JvmField
        var X_MULTIPLIER: Double = 0.49 // Multiplier in the X direction

        @JvmField
        var Y_MULTIPLIER: Double = 0.49 // Multiplier in the Y direction

        fun encoderTicksToInches(ticks: Double): Double = WHEEL_RADIUS * 2 * Math.PI * GEAR_RATIO * ticks / TICKS_PER_REV
    }
}
