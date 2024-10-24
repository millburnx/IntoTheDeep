package org.firstinspires.ftc.teamcode.common.subsystems

import com.acmerobotics.dashboard.config.Config
import com.arcrobotics.ftclib.command.SubsystemBase
import com.arcrobotics.ftclib.controller.PIDController
import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.DcMotorEx
import com.qualcomm.robotcore.hardware.DcMotorSimple
import com.qualcomm.robotcore.hardware.HardwareMap
import org.firstinspires.ftc.robotcore.external.Telemetry
import kotlin.math.cos

@Config
class Arm(hardwareMap: HardwareMap, val telemetry: Telemetry, val liftPosition: () -> Int) : SubsystemBase() {
    val leftRotate: DcMotorEx by lazy {
        hardwareMap["leftRotate"] as DcMotorEx
    }

    val rightRotate: DcMotorEx by lazy {
        hardwareMap["rightRotate"] as DcMotorEx
    }

    val controller: PIDController = PIDController(p, i, d)
    var target: Int = 0

    val position: Double
        get() = rightRotate.currentPosition + starting_ticks

    val angle: Double
        get() = position / ticks_in_degree

    init {
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

    fun run(telemetry: Telemetry) {
        val newP = p + p * (liftPosition() * slidePMulti); // p + mp
        controller.setPID(newP, i, d)
        val modifier = if (target < position) {
            downMulti
        } else {
            1.0
        }
        val pid = controller.calculate(position.toDouble(), target.toDouble()) * modifier
        val finalF = f + f * (liftPosition() * slideFMulti) // f + mf
        val ffAngle = if (realtimeFF) angle else (target.toDouble() / ticks_in_degree)
        val ff = cos(Math.toRadians(ffAngle)) * finalF

        val power = pid + ff

        leftRotate.power = -power
        rightRotate.power = -power

        telemetry.addData("arm power", power)
        telemetry.addData("arm pid", pid)
        telemetry.addData("arm ff", ff)
        telemetry.addData("arm error", target.toDouble() - position)
    }

    companion object {
        @JvmField
        var p: Double = 0.03

        @JvmField
        var i: Double = 0.0

        @JvmField
        var d: Double = 0.0

        @JvmField
        var f: Double = 0.4

        @JvmField
        var base: Int = 30

        @JvmField
        var pickup: Int = 0

        @JvmField
        var lowBasket: Int = 170

        @JvmField
        var highBasket: Int = 170

        @JvmField
        var ticks_in_degree: Double = 160.0 / 90.0

        @JvmField
        var starting_ticks: Double = 15.0

        @JvmField
        var downMulti: Double = 0.175

        @JvmField
        var slidePMulti: Double = 0.001

        @JvmField
        var slideFMulti: Double = 0.001

        @JvmField
        var realtimeFF: Boolean = false
    }
}