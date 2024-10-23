package org.firstinspires.ftc.teamcode.common.subsystems

import com.acmerobotics.dashboard.config.Config
import com.arcrobotics.ftclib.controller.PIDController
import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.DcMotorEx
import com.qualcomm.robotcore.hardware.DcMotorSimple
import com.qualcomm.robotcore.hardware.HardwareMap
import org.firstinspires.ftc.robotcore.external.Telemetry
import kotlin.math.cos

@Config
class ArmPID(hardwareMap: HardwareMap, val liftPosition: () -> Int) {
    var leftRotate: DcMotorEx

    @JvmField
    var rightRotate: DcMotorEx

    private val controller: PIDController
    var target: Int = 0

    val angle: Double
        get() = rightRotate.currentPosition / ticks_in_degree

    init {
        controller = PIDController(p, i, d)
        leftRotate = hardwareMap["leftRotate"] as DcMotorEx
        leftRotate.direction = DcMotorSimple.Direction.FORWARD
        leftRotate.zeroPowerBehavior = DcMotor.ZeroPowerBehavior.BRAKE
        leftRotate.mode = DcMotor.RunMode.STOP_AND_RESET_ENCODER
        leftRotate.mode = DcMotor.RunMode.RUN_WITHOUT_ENCODER

        rightRotate = hardwareMap["rightRotate"] as DcMotorEx
        rightRotate.direction = DcMotorSimple.Direction.REVERSE
        rightRotate.zeroPowerBehavior = DcMotor.ZeroPowerBehavior.BRAKE
        rightRotate.mode = DcMotor.RunMode.STOP_AND_RESET_ENCODER
        rightRotate.mode = DcMotor.RunMode.RUN_WITHOUT_ENCODER
    }

    fun run(telemetry: Telemetry) {
        val newP = p + p * (liftPosition() * slidePMulti);
        controller.setPID(newP, i, d)
        val pos = rightRotate.currentPosition + starting_ticks
        val modifier = if (target < pos) {
            downMulti
        } else {
            1.0
        }
        val pid = controller.calculate(pos.toDouble(), target.toDouble()) * modifier
        val finalF = f + f * (liftPosition() * slideFMulti) // f + mf
        val ffAngle = (if (realtimeFF) pos else target).toDouble() / ticks_in_degree
        val ff = cos(Math.toRadians(ffAngle)) * finalF

        val power = pid + ff

        leftRotate.power = -power
        rightRotate.power = -power

        telemetry.addData("arm power", -power)
        telemetry.addData("arm pid", pid)
        telemetry.addData("arm ff", ff)
        telemetry.addData("arm error", target.toDouble() - pos.toDouble())
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
        var floor: Int = 30

        @JvmField
        var up: Int = 120

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