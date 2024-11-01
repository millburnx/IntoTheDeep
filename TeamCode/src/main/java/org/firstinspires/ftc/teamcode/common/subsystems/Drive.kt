package org.firstinspires.ftc.teamcode.common.subsystems

import com.acmerobotics.dashboard.FtcDashboard
import com.acmerobotics.dashboard.config.Config
import com.arcrobotics.ftclib.command.SubsystemBase
import com.arcrobotics.ftclib.geometry.Pose2d
import com.arcrobotics.ftclib.hardware.motors.Motor
import com.arcrobotics.ftclib.hardware.motors.MotorEx
import com.qualcomm.hardware.lynx.LynxModule
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot
import com.qualcomm.robotcore.hardware.HardwareMap
import com.qualcomm.robotcore.hardware.IMU
import org.firstinspires.ftc.teamcode.common.utils.PoseColor
import org.firstinspires.ftc.teamcode.common.utils.Telemetry
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive
import kotlin.math.abs
import kotlin.math.cos
import kotlin.math.max
import kotlin.math.sin

@Config
class Drive(
    hardwareMap: HardwareMap,
    val telemetry: Telemetry,
    val dashboard: FtcDashboard,
    val cacheThreshold: Double = -1.0
) : SubsystemBase() {
    private val prevPower = doubleArrayOf(0.0, 0.0, 0.0, 0.0)
    val leftFront: MotorEx by lazy {
        MotorEx(hardwareMap, "frontLeft")
    }
    val leftRear: MotorEx by lazy {
        MotorEx(hardwareMap, "backLeft")
    }
    val rightRear: MotorEx by lazy {
        MotorEx(hardwareMap, "backRight")
    }
    val rightFront: MotorEx by lazy {
        MotorEx(hardwareMap, "frontRight")
    }

    val imu: IMU by lazy {
        hardwareMap["imu"] as IMU
    }
    val rrDrive: SampleMecanumDrive by lazy {
        SampleMecanumDrive(hardwareMap)
    }

    val leftOdom: Motor.Encoder by lazy {
        leftRear.encoder
    }
    val centerOdom: Motor.Encoder by lazy {
        leftFront.encoder
    }
    val rightOdom: Motor.Encoder by lazy {
        rightFront.encoder
    }
    val pose: Pose2d
        get() = rrDrive.pose


    init {
        DISTANCE_PER_PULSE = Math.PI * WHEEL_DIAMETER / TICKS_PER_REV

        leftFront.inverted = true
        leftRear.inverted = true
        rightRear.inverted = false
        rightFront.inverted = false

        val stop = if (BREAK) Motor.ZeroPowerBehavior.BRAKE else Motor.ZeroPowerBehavior.FLOAT

        rightRear.setZeroPowerBehavior(stop)
        rightFront.setZeroPowerBehavior(stop)
        leftRear.setZeroPowerBehavior(stop)
        leftFront.setZeroPowerBehavior(stop)

        imu.initialize(
            IMU.Parameters(
                RevHubOrientationOnRobot(
                    RevHubOrientationOnRobot.LogoFacingDirection.RIGHT,
                    RevHubOrientationOnRobot.UsbFacingDirection.UP
                )
            )
        )
        imu.resetYaw()

        leftOdom.setDistancePerPulse(DISTANCE_PER_PULSE)
        centerOdom.setDistancePerPulse(DISTANCE_PER_PULSE)
        rightOdom.setDistancePerPulse(DISTANCE_PER_PULSE)

        leftOdom.setDirection(Motor.Direction.REVERSE)
        centerOdom.setDirection(Motor.Direction.FORWARD)
        rightOdom.setDirection(Motor.Direction.FORWARD)

        leftOdom.reset()
        centerOdom.reset()
        rightOdom.reset()

        // change to reflect starting field position

        val allHubs = hardwareMap.getAll<LynxModule?>(LynxModule::class.java)

        for (hub in allHubs) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO)
        }
    }

    override fun periodic() {
        rrDrive.updatePoseEstimate()
        telemetry.drawField(
            listOf(
                PoseColor(
                    rrDrive.pose, "#0000ff"
                )
            ), dashboard
        )
    }

    fun robotCentric(power: Double, strafe: Double, turn: Double, multiF: Double = 1.0, multiH: Double = 1.0) {
        val denominator = max(
            abs(power * 1 / multiF) + abs(strafe * 1 / multiF) + abs(turn * 1 / multiH),
            max(1.0, max(1 / multiF, 1 / multiH))
        )
        val frontLeftPower = (power + strafe + turn) / denominator
        val backLeftPower = (power - strafe + turn) / denominator
        val frontRightPower = (power - strafe - turn) / denominator
        val backRightPower = (power + strafe - turn) / denominator

        val caching = cacheThreshold == -1.0
        if (caching) {
            val frontLeftDiff = abs(prevPower[0] - frontLeftPower)
            val backLeftDiff = abs(prevPower[1] - backLeftPower)
            val frontRightDiff = abs(prevPower[2] - frontRightPower)
            val backRightDiff = abs(prevPower[3] - backRightPower)
            if (frontLeftDiff > cacheThreshold) {
                prevPower[0] = frontLeftPower
                leftFront.set(frontLeftPower)
            }
            if (backLeftDiff > cacheThreshold) {
                prevPower[1] = backLeftPower
                leftRear.set(backLeftPower)
            }
            if (frontRightDiff > cacheThreshold) {
                prevPower[2] = frontRightPower
                rightFront.set(frontRightPower)
            }
            if (backRightDiff > cacheThreshold) {
                prevPower[3] = backRightPower
                rightRear.set(backRightPower)
            }
        } else {
            leftFront.set(frontLeftPower)
            leftRear.set(backLeftPower)
            rightFront.set(frontRightPower)
            rightRear.set(backRightPower)
        }
    }

    fun fieldCentric(
        x: Double,
        y: Double,
        rx: Double,
        heading: Double,
        tel: org.firstinspires.ftc.robotcore.external.Telemetry? = null
    ) {
        // Rotate the movement direction counter to the bot's rotation
        var rotX = x * cos(heading) - y * sin(heading)
        val rotY = x * sin(heading) + y * cos(heading)

        rotX = rotX * 1.1

        if (tel != null) {
            tel.addData("rotX", rotX)
            tel.addData("rotY", rotY)
        }
        if (applyPower) robotCentric(rotX, rotY, rx)
    }

    companion object {
        @JvmField
        var applyPower: Boolean = true

        @JvmField
        var TRACK_WIDTH: Double = -12.5

        @JvmField
        var CENTER_WHEEL_OFFSET: Double = 1.75
        // distance between center of rotation of the robot and the center odometer

        @JvmField
        var WHEEL_DIAMETER: Double = 1.425

        @JvmField
        var TICKS_PER_REV: Double = 8192.0

        @JvmField
        var DISTANCE_PER_PULSE: Double = Math.PI * WHEEL_DIAMETER / TICKS_PER_REV

        @JvmField
        var BREAK: Boolean = false

        @JvmField
        var startingX: Double = 0.0

        @JvmField
        var startingY: Double = 0.0

        @JvmField
        var startingH: Double = 0.0
    }
}
