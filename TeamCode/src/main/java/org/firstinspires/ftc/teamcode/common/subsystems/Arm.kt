package org.firstinspires.ftc.teamcode.common.subsystems

import com.acmerobotics.dashboard.config.Config
import com.arcrobotics.ftclib.command.SubsystemBase
import com.arcrobotics.ftclib.controller.PIDController
import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.DcMotorEx
import com.qualcomm.robotcore.hardware.DcMotorSimple
import com.qualcomm.robotcore.hardware.HardwareMap
import org.firstinspires.ftc.robotcore.external.Telemetry
import org.firstinspires.ftc.teamcode.common.utils.LimitedDurationQueue
import kotlin.math.abs
import kotlin.math.cos

@Config
class Arm(
    hardwareMap: HardwareMap,
    val telemetry: Telemetry,
    val liftPosition: () -> Int,
    val startingOffset: Double = starting_ticks,
) : SubsystemBase() {

    val leftRotate: DcMotorEx by lazy {
        hardwareMap["leftRotate"] as DcMotorEx
    }

    val previousArmPosition = LimitedDurationQueue<Double>(lifespan, satThreshold)

    var isOverride = false

    var isOn = false
        set(value) {
            if (!field && value) {
                previousArmPosition.clear()
                currentMaxPid = maxPid
            }
            field = value
        }

    val rightRotate: DcMotorEx by lazy {
        hardwareMap["rightRotate"] as DcMotorEx
    }

    val controller: PIDController = PIDController(p, i, d)
    var target: Double = 0.0
        set(value) {
            if (value != field) {
                currentMaxPid = maxPid
            }
            field = value.coerceAtMost(max)
        }

    val position: Double
        get() = rightRotate.currentPosition + startingOffset

    val angle: Double
        get() = position / ticks_in_degree

    var currentMaxPid = maxPid

    init {
        resetEncoders()
    }

    fun resetEncoders() {
        leftRotate.direction = DcMotorSimple.Direction.FORWARD
        leftRotate.zeroPowerBehavior = DcMotor.ZeroPowerBehavior.FLOAT
        leftRotate.mode = DcMotor.RunMode.STOP_AND_RESET_ENCODER
        leftRotate.mode = DcMotor.RunMode.RUN_WITHOUT_ENCODER

        rightRotate.direction = DcMotorSimple.Direction.REVERSE
        rightRotate.zeroPowerBehavior = DcMotor.ZeroPowerBehavior.FLOAT
        rightRotate.mode = DcMotor.RunMode.STOP_AND_RESET_ENCODER
        rightRotate.mode = DcMotor.RunMode.RUN_WITHOUT_ENCODER
    }

    override fun periodic() {
        run(telemetry)
        previousArmPosition.lifespan = lifespan
        previousArmPosition.threshold = satThreshold
        previousArmPosition.add(position)
        val error = target - position
        val change = getChange()
        if (change != null) {
            currentMaxPid += abs(error / change.coerceAtLeast(0.0001) * increaseRate)
            println("$change, $error, ${error / change.coerceAtLeast(0.0001) * increaseRate}, $currentMaxPid")
        }
    }

    fun getChange(): Double? {
        if (!previousArmPosition.isSaturated()) return null
        val first = previousArmPosition.values.first()
        val last = previousArmPosition.values.last()
        return first - last
    }

    fun setPower(power: Double) {
        leftRotate.power = -power
        rightRotate.power = -power
    }

    fun run(telemetry: Telemetry) {
        if (isOverride) {
            return // manual power
        }

        if (!isOn) {
            setPower(0.0)
            return
        }

        if (position > max) {
            // MOVE THE ARM DOWN
            setPower(-kCos / 2)
            return
        }

        val newP = p + p * (liftPosition() * slidePMulti); // p + mp
        controller.setPID(newP, i, d)
        val modifier = if (target < position) {
            downMulti + downMulti * downSlideMulti
        } else {
            1.0
        }

        val finalKCos = kCos + kCos * (liftPosition() * slideKCosMulti) // f + mf
        val ffAngle = if (realtimeFF) angle else (target / ticks_in_degree)

        // ff gets slow around 45 so we increase
        val extraFF = kTF / (abs(angle - kTFAngle) + 1)

        val ff =
            extraFF + extraFF * (liftPosition() * slideKTFMulti) + kG + kG * (liftPosition() * slideKGMulti) + cos(
                Math.toRadians(ffAngle)
            ) * finalKCos


        val pid = controller.calculate(position, target) * modifier
        val maxPossiblePower = (1 - abs(ff)).coerceIn(0.0, currentMaxPid + currentMaxPid * slidePMulti)
        val clampedPid = pid.coerceIn(-(maxPossiblePower), maxPossiblePower)

        val power = ff + clampedPid

        setPower(power)

        telemetry.addData("arm power", power)
        telemetry.addData("arm pid", clampedPid)
        telemetry.addData("arm ff", ff)
        telemetry.addData("arm error", target - position)
        telemetry.addData("max pid", currentMaxPid)
        telemetry.addData("arm size", previousArmPosition.values.size)
        telemetry.addData("arm first", previousArmPosition.values.firstOrNull())
        telemetry.addData("arm last", previousArmPosition.values.lastOrNull())
    }

    fun off() {
        isOn = false
    }

    fun on() {
        isOn = true
    }

    companion object {
        @JvmField
        var lifespan: Double = 200.0

        @JvmField
        var satThreshold: Double = 70.0

        @JvmField
        var armOnDelay: Long = 50

        @JvmField
        var p: Double = 0.015

        @JvmField
        var i: Double = 0.0

        @JvmField
        var d: Double = 0.0

        @JvmField
        var kG: Double = 0.3125

        @JvmField
        var kCos: Double = -0.1

        @JvmField
        var kTFAngle: Double = 50.0

        @JvmField
        var kTF: Double = 0.0

        @JvmField
        var base: Int = 45

        @JvmField
        var ticks_in_degree: Double = 160.0 / 90.0

        @JvmField
        var starting_ticks: Double = 0.0

        @JvmField
        var downMulti: Double = 1.0

        @JvmField
        var downSlideMulti: Double = 0.0

        @JvmField
        var slideKCosMulti: Double = 0.0

        @JvmField
        var slideKGMulti: Double = 0.00125

        @JvmField
        var slideKTFMulti: Double = 0.0

        @JvmField
        var slidePMulti: Double = 0.002

        @JvmField
        var realtimeFF: Boolean = false

        @JvmField
        var threshold: Int = 10

        @JvmField
        var maxPid: Double = 0.1

        @JvmField
        var max: Double = 160.0

        @JvmField
        var increaseRate: Double = 1e-7

        @JvmField
        var submersible: Int = 30
    }
}