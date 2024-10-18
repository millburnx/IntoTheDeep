package org.firstinspires.ftc.teamcode.common.subsystems

import com.acmerobotics.dashboard.config.Config
import com.arcrobotics.ftclib.command.SubsystemBase
import com.arcrobotics.ftclib.geometry.Pose2d
import com.arcrobotics.ftclib.geometry.Rotation2d
import com.arcrobotics.ftclib.hardware.motors.Motor
import com.arcrobotics.ftclib.hardware.motors.Motor.Direction
import com.arcrobotics.ftclib.hardware.motors.Motor.ZeroPowerBehavior
import com.arcrobotics.ftclib.hardware.motors.MotorEx
import com.arcrobotics.ftclib.kinematics.HolonomicOdometry
import com.qualcomm.hardware.lynx.LynxModule
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot
import com.qualcomm.robotcore.hardware.HardwareMap
import com.qualcomm.robotcore.hardware.IMU
import kotlin.math.abs
import kotlin.math.cos
import kotlin.math.sin

@Config
object DriveConfig {
    @JvmField
    val breakStop = false

    @JvmField
    val WHEEL_DIAMETER = 1.45

    @JvmField
    val TICKS_PER_REV = 8192

    @JvmField
    val DISTANCE_PER_PULSE = Math.PI * WHEEL_DIAMETER / TICKS_PER_REV

    @JvmField
    val TRACK_WIDTH = -12.375

    @JvmField
    val CENTER_WHEEL_OFFSET = 0.5

    @JvmField
    val startingX = -60.0

    @JvmField
    val startingY = -60.0

    @JvmField
    val startingH = 0.0
}

class Drive(hardwareMap: HardwareMap, val motorCacheThreshold: Double) : SubsystemBase() {
    val prevPower = mutableListOf(0.0, 0.0, 0.0, 0.0)
    val stopMode = if (DriveConfig.breakStop) ZeroPowerBehavior.BRAKE else ZeroPowerBehavior.FLOAT
    val leftFront: MotorEx by lazy {
        val leftFront = MotorEx(hardwareMap, "frontLeft")
        leftFront.inverted = true
        leftFront.setZeroPowerBehavior(stopMode)
        return@lazy leftFront
    }
    val leftRear: MotorEx by lazy {
        val leftRear = MotorEx(hardwareMap, "backLeft")
        leftRear.inverted = true
        leftRear.setZeroPowerBehavior(stopMode)
        return@lazy leftRear
    }
    val rightFront: MotorEx by lazy {
        val rightFront = MotorEx(hardwareMap, "frontLeft")
        rightFront.inverted = false
        rightFront.setZeroPowerBehavior(stopMode)
        return@lazy rightFront
    }
    val rightRear: MotorEx by lazy {
        val rightRear = MotorEx(hardwareMap, "backLeft")
        rightRear.inverted = false
        rightRear.setZeroPowerBehavior(stopMode)
        return@lazy rightRear
    }
    val imu: IMU by lazy {
        val imu = hardwareMap["imu"] as IMU
        val params = IMU.Parameters(
            RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.LEFT,
                RevHubOrientationOnRobot.UsbFacingDirection.UP
            )
        )
        imu.initialize(params)
        imu.resetYaw()
        return@lazy imu
    }
    val leftOdom: Motor.Encoder by lazy {
        val leftOdom = rightRear.encoder.setDistancePerPulse(DriveConfig.DISTANCE_PER_PULSE)
        leftOdom.setDirection(Direction.FORWARD)
        leftOdom.reset()
        return@lazy leftOdom
    }
    val rightOdom: Motor.Encoder by lazy {
        val rightOdom = leftFront.encoder.setDistancePerPulse(DriveConfig.DISTANCE_PER_PULSE)
        rightOdom.setDirection(Direction.REVERSE)
        rightOdom.reset()
        return@lazy rightOdom
    }
    val centerOdom: Motor.Encoder by lazy {
        val centerOdom = leftRear.encoder.setDistancePerPulse(DriveConfig.DISTANCE_PER_PULSE)
        centerOdom.setDirection(Direction.REVERSE)
        centerOdom.reset()
        return@lazy centerOdom
    }
    val odometry: HolonomicOdometry by lazy {
        val odom = HolonomicOdometry(
            leftOdom::getDistance,
            rightOdom::getDistance,
            centerOdom::getDistance,
            DriveConfig.TRACK_WIDTH, DriveConfig.CENTER_WHEEL_OFFSET
        )
        odom.updatePose(
            Pose2d(
                DriveConfig.startingX,
                DriveConfig.startingY,
                Rotation2d(Math.toRadians(DriveConfig.startingH))
            )
        )
        return@lazy odom
    }

    init {
        val allHubs: List<LynxModule> = hardwareMap.getAll(LynxModule::class.java)

        for (hub in allHubs) {
            hub.bulkCachingMode = LynxModule.BulkCachingMode.AUTO
        }
    }

    fun robotCentric(power: Double, strafe: Double, turn: Double, multiF: Double = 1.0, multiH: Double = 1.0) {
        val denominator = maxOf(
            (abs(power / multiF) + abs(strafe / multiF) + abs(turn / multiH)),
            maxOf(1.0, maxOf(1 / multiF, 1 / multiH))
        )
        val frontLeftPower = (power + strafe + turn) / denominator
        val backLeftPower = (power - strafe + turn) / denominator
        val frontRightPower = (power - strafe - turn) / denominator
        val backRightPower = (power + strafe - turn) / denominator

        val caching = motorCacheThreshold == -1.0
        if (caching) {
            val frontLeftDiff = abs(prevPower[0] - frontLeftPower)
            val backLeftDiff = abs(prevPower[1] - backLeftPower)
            val frontRightDiff = abs(prevPower[2] - frontRightPower)
            val backRightDiff = abs(prevPower[3] - backRightPower)
            if (frontLeftDiff > motorCacheThreshold) {
                prevPower[0] = frontLeftPower
                leftFront.set(frontLeftPower)
            }
            if (backLeftDiff > motorCacheThreshold) {
                prevPower[1] = backLeftPower
                leftRear.set(backLeftPower)
            }
            if (frontRightDiff > motorCacheThreshold) {
                prevPower[2] = frontRightPower
                rightFront.set(frontRightPower)
            }
            if (backRightDiff > motorCacheThreshold) {
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

    fun fieldCentric(x: Double, y: Double, rx: Double, heading: Double) {
        // Rotate the movement direction counter to the bot's rotation
        val rotX = (x * cos(-heading) - y * sin(-heading)) * 1.1
        val rotY = x * sin(-heading) + y * cos(-heading)

        robotCentric(rotY, rotX, rx)
    }

    fun updatePos() {
        odometry.updatePose()
    }

    fun getPos(): Pose2d {
        return odometry.pose
    }
}