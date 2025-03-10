package org.firstinspires.ftc.teamcode.common.subsystems.drive

import com.acmerobotics.dashboard.config.Config
import com.qualcomm.robotcore.hardware.HardwareMap
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D
import org.firstinspires.ftc.teamcode.common.utils.Pose2d
import org.firstinspires.ftc.teamcode.common.utils.Subsystem
import org.firstinspires.ftc.teamcode.common.utils.hardware.GoBildaPinpointDriver

@Config
class PinPoint(
    val hardwareMap: HardwareMap,
    val deviceName: String,
) : Subsystem() {
    val pinPoint = hardwareMap[deviceName] as GoBildaPinpointDriver

    init {
        val circumferenceMM = diameterMM * Math.PI
        val ticksPerMM = ticksPerRevolution / circumferenceMM

        pinPoint.setEncoderResolution(ticksPerMM)

        pinPoint.setOffsets(xOffset * 25.4, yOffset * 25.4)
        pinPoint.setEncoderDirections(
            GoBildaPinpointDriver.EncoderDirection.FORWARD,
            GoBildaPinpointDriver.EncoderDirection.REVERSED,
        )
    }

    override fun init() {
//        reset()
    }

    override fun periodic() {
        update()
    }

    fun update() = pinPoint.update()

    fun headingUpdate() = pinPoint.update(GoBildaPinpointDriver.readData.ONLY_UPDATE_HEADING)

    fun reset() = pinPoint.resetPosAndIMU()

    fun recalibrateImu() = pinPoint.recalibrateIMU()

    var pose: Pose2d = Pose2d()
        get() {
            val position = pinPoint.position
            return Pose2d(
                position.getX(DistanceUnit.INCH),
                position.getY(DistanceUnit.INCH),
                position.getHeading(AngleUnit.DEGREES),
            )
        }
        set(value) {
            if (field == value) return
            val ftcPose = Pose2D(DistanceUnit.INCH, value.x, value.y, AngleUnit.DEGREES, value.heading)
            pinPoint.setPosition(ftcPose)
            println(ftcPose)
            field = value
        }

    val velocity: Pose2d
        get() {
            val velocity = pinPoint.velocity
            return Pose2d(
                velocity.getX(DistanceUnit.INCH),
                velocity.getY(DistanceUnit.INCH),
                velocity.getHeading(AngleUnit.DEGREES),
            )
        }

    val status
        get() = pinPoint.getDeviceStatus()

    val hertz
        get() = pinPoint.frequency

    companion object {
        @JvmField
        var diameterMM: Double = 35.0 // 35mm (open odo)

        @JvmField
        var ticksPerRevolution: Double = 8192.0 // rev through bore

        @JvmField
        var xOffset: Double = 5.75

        @JvmField
        var yOffset: Double = -.75
    }
}
