package org.firstinspires.ftc.teamcode.common.commands

import com.acmerobotics.dashboard.config.Config
import com.arcrobotics.ftclib.command.CommandBase
import com.arcrobotics.ftclib.gamepad.GamepadEx
import org.firstinspires.ftc.robotcore.external.Telemetry
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit
import org.firstinspires.ftc.teamcode.common.subsystems.Drive
import org.firstinspires.ftc.teamcode.opmodes.teleop.MainTelelop.Companion.fieldCentric
import org.firstinspires.ftc.teamcode.opmodes.teleop.MainTelelop.Companion.flipY
import kotlin.math.abs
import kotlin.math.cbrt
import kotlin.math.sign


@Config
class DriveRobotCommand(
    val drive: Drive,
    val gamepad: GamepadEx,
    val telemetry: Telemetry,
    val isSlowMode: () -> Boolean,
    val cubic: () -> Boolean
) : CommandBase() {
    init {
        addRequirements(drive)
    }

    override fun execute() {
        val multi = if (isSlowMode()) slowSpeed else regSpeed

        val power =
            (if (cubic()) cbrt(abs(gamepad.leftY)) * sign(gamepad.leftY) else gamepad.leftY) * multi * if (flipY) -1.0 else 1.0
        val strafe = (if (cubic()) cbrt(abs(gamepad.leftX)) * sign(gamepad.leftX) else gamepad.leftX) * multi * 1.1
        val turn = (if (cubic()) cbrt(abs(gamepad.rightX)) * sign(gamepad.rightX) else gamepad.rightX) * multi

        if (!fieldCentric) {
            drive.robotCentric(power, strafe, turn)
        } else {
            val heading = drive.imu.robotYawPitchRollAngles.getYaw(AngleUnit.RADIANS)
            telemetry.addData("heading (imu)", Math.toDegrees(heading))
            drive.fieldCentric(power, strafe, turn, heading + Math.toRadians(Drive.Companion.startingH))
        }
    }

    override fun isFinished(): Boolean {
        return true;
    }

    companion object {
        @JvmField
        var regSpeed: Double = 0.5

        @JvmField
        var slowSpeed: Double = 0.3
    }
}