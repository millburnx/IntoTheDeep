package org.firstinspires.ftc.teamcode.opmodes.tuning

import com.acmerobotics.dashboard.config.Config
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import com.qualcomm.robotcore.hardware.DcMotorEx
import org.firstinspires.ftc.teamcode.common.Robot
import org.firstinspires.ftc.teamcode.common.subsystems.drive.Drive
import org.firstinspires.ftc.teamcode.common.utils.OpMode
import org.firstinspires.ftc.teamcode.common.utils.Subsystem

class OdomOnly(opmode: OpMode) : Robot(opmode) {
    override val drive: Drive by lazy { Drive(this) }
    override val subsystems: List<Subsystem> by lazy { listOf(drive) }
}

@TeleOp(name = "Odom Tuner")
@Config
class OdomTuner : OpMode() {
    override val robot by lazy { OdomOnly(this) }

    val para: DcMotorEx by lazy { robot.hardware["para"] as DcMotorEx }
    val perp: DcMotorEx by lazy { robot.hardware["perp"] as DcMotorEx }

    override fun exec() {
        robot.drive.robotCentric(
            gamepad1.left_stick_y.toDouble(),
            -gamepad1.left_stick_x.toDouble(),
            -gamepad1.right_stick_x.toDouble()
        )

        val pose = robot.drive.pose
        robot.telemetry.addData("para: ", para.currentPosition)
        robot.telemetry.addData("perp: ", perp.currentPosition)
        robot.telemetry.addData("x: ", pose.x)
        robot.telemetry.addData("y: ", pose.y)
        robot.telemetry.addData("h: ", Math.toDegrees(pose.heading))
    }
}