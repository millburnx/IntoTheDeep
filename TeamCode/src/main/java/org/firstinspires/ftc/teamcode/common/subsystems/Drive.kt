package org.firstinspires.ftc.teamcode.common.subsystems

import com.qualcomm.robotcore.hardware.DcMotor.RunMode
import com.qualcomm.robotcore.hardware.DcMotor.ZeroPowerBehavior
import com.qualcomm.robotcore.hardware.DcMotorEx
import com.qualcomm.robotcore.hardware.DcMotorSimple.Direction
import org.firstinspires.ftc.teamcode.common.Robot
import org.firstinspires.ftc.teamcode.common.utils.Subsystem
import kotlin.math.abs
import kotlin.math.max

class Drive(robot: Robot) : Subsystem() {
    val frontLeft: DcMotorEx = (robot.hardware["frontLeft"] as DcMotorEx).also {
        it.direction = Direction.FORWARD
    }
    val frontRight: DcMotorEx = (robot.hardware["frontRight"] as DcMotorEx).also {
        it.direction = Direction.FORWARD
    }
    val backLeft: DcMotorEx = (robot.hardware["backLeft"] as DcMotorEx).also {
        it.direction = Direction.FORWARD
    }
    val backRight: DcMotorEx = (robot.hardware["backRight"] as DcMotorEx).also {
        it.direction = Direction.FORWARD
    }
    val motors = listOf<DcMotorEx>(frontLeft, frontRight, backLeft, backRight).also {
        it.forEach {
            it.zeroPowerBehavior = ZeroPowerBehavior.FLOAT
            it.mode = RunMode.STOP_AND_RESET_ENCODER
            it.mode = RunMode.RUN_WITHOUT_ENCODER
        }
    }

    fun robotCentric(forward: Double, strafe: Double, rotate: Double) {
        val denominator = max(
            abs(forward) + abs(strafe) + abs(rotate), 1.0
        )
        val frontLeftPower = (forward + strafe + rotate) / denominator
        val frontRightPower = (forward - strafe - rotate) / denominator
        val backLeftPower = (forward - strafe + rotate) / denominator
        val backRightPower = (forward + strafe - rotate) / denominator

        frontLeft.power = frontLeftPower
        frontRight.power = frontRightPower
        backLeft.power = backLeftPower
        backRight.power = backRightPower
    }
}