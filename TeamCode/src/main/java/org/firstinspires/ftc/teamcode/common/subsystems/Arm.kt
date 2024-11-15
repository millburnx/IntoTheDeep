package org.firstinspires.ftc.teamcode.common.subsystems

import com.acmerobotics.dashboard.config.Config
import com.arcrobotics.ftclib.command.SubsystemBase
import com.arcrobotics.ftclib.controller.PIDController
import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.DcMotorEx
import com.qualcomm.robotcore.hardware.DcMotorSimple
import com.qualcomm.robotcore.hardware.HardwareMap
import org.firstinspires.ftc.robotcore.external.Telemetry
import kotlin.math.abs
import kotlin.math.cos

@Config
class Arm(hardwareMap: HardwareMap, val telemetry: Telemetry, val liftPosition: () -> Int) : SubsystemBase() {
    val leftRotate: DcMotorEx by lazy {
        hardwareMap["leftRotate"] as DcMotorEx
    }

    var isOverride = false

    var isOn = false

    val rightRotate: DcMotorEx by lazy {
        hardwareMap["rightRotate"] as DcMotorEx
    }

    val controller: PIDController = PIDController(p, i, d)
    var target: Double = 0.0
        set(value) {
            field = value.coerceAtMost(max)
        }

    val position: Double
        get() = rightRotate.currentPosition + starting_ticks

    val angle: Double
        get() = position / ticks_in_degree

    init {
        resetEncoders()
    }

    fun resetEncoders() {
        leftRotate.direction = DcMotorSimple.Direction.FORWARD
        leftRotate.zeroPowerBehavior = DcMotor.ZeroPowerBehavior.BRAKE
        leftRotate.mode = DcMotor.RunMode.STOP_AND_RESET_ENCODER
        leftRotate.mode = DcMotor.RunMode.RUN_WITHOUT_ENCODER

        rightRotate.direction = DcMotorSimple.Direction.REVERSE
        rightRotate.zeroPowerBehavior = DcMotor.ZeroPowerBehavior.BRAKE
        rightRotate.mode = DcMotor.RunMode.STOP_AND_RESET_ENCODER
        rightRotate.mode = DcMotor.RunMode.RUN_WITHOUT_ENCODER
    }

    override fun periodic() {
        run(telemetry)
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
        val ffAngle = if (realtimeFF) angle else (target.toDouble() / ticks_in_degree)

        // ff gets slow around 45 so we increase
        val extraFF = kTF / (abs(angle - kTFAngle) + 1)

        val ff = extraFF + extraFF * (liftPosition() * slideKTFMulti) + kG + kG * (liftPosition() * slideKGMulti) + cos(
            Math.toRadians(ffAngle)
        ) * finalKCos


        val pid = controller.calculate(position.toDouble(), target.toDouble()) * modifier
        val maxPossiblePower = (1 - abs(ff)).coerceIn(0.0, maxPid * slidePMulti)
        val clampedPid = pid.coerceIn(-(maxPossiblePower), maxPossiblePower)

        val power = ff + clampedPid

        setPower(power)

        telemetry.addData("arm power", power)
        telemetry.addData("arm pid", clampedPid)
        telemetry.addData("arm ff", ff)
        telemetry.addData("arm error", target.toDouble() - position)
    }

    fun off() {
        isOn = false
    }

    fun on() {
        isOn = true
    }

    companion object {
        @JvmField
        var p: Double = 0.015

        @JvmField
        var i: Double = 0.0

        @JvmField
        var d: Double = 0.0

        @JvmField
        var kG: Double = 0.29

        @JvmField
        var kCos: Double = -0.1

        @JvmField
        var kTFAngle: Double = 50.0

        @JvmField
        var kTF: Double = 0.0

        @JvmField
        var base: Int = 30

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
        var maxPid: Double = 0.2

        @JvmField
        var max: Double = 160.0
    }
}