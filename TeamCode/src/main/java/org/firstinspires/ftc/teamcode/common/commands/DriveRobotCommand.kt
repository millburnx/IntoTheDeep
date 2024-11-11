package org.firstinspires.ftc.teamcode.common.commands

import com.acmerobotics.dashboard.config.Config
import com.arcrobotics.ftclib.command.CommandBase
import org.firstinspires.ftc.robotcore.external.Telemetry
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit
import org.firstinspires.ftc.teamcode.common.subsystems.Drive
import org.firstinspires.ftc.teamcode.common.utils.GamepadSRL
import org.firstinspires.ftc.teamcode.opmodes.teleop.MainTelelop.Companion.fieldCentric
import org.firstinspires.ftc.teamcode.opmodes.teleop.MainTelelop.Companion.flipY
import kotlin.math.abs
import kotlin.math.sign
import kotlin.math.sqrt


@Config
class DriveRobotCommand(
    val drive: Drive,
    val gamepad: GamepadSRL,
    val telemetry: Telemetry,
    val isSlowMode: () -> Boolean,
    val cubic: () -> Boolean
) : CommandBase() {
    init {
        addRequirements(drive)
    }

    override fun execute() {
        val multi = if (isSlowMode()) slowSpeed else regSpeed
        false
        val power =
            (if (cubic()) sqrt(abs(gamepad.leftStick.y)) * sign(gamepad.leftStick.y) else gamepad.leftStick.y) * multi * if (flipY) -1.0 else 1.0
        val strafe =
            (if (cubic()) sqrt(abs(gamepad.leftStick.x)) * sign(gamepad.leftStick.x) else gamepad.leftStick.x) * multi * 1.1
        val turn =
            (if (cubic()) sqrt(abs(gamepad.rightStick.x)) * sign(gamepad.rightStick.x) else gamepad.rightStick.x) * multi

        if (!fieldCentric) {
            drive.robotCentric(power, strafe, turn)
        } else {
            val heading = drive.imu.robotYawPitchRollAngles.getYaw(AngleUnit.RADIANS)
            telemetry.addData("heading (imu)", Math.toDegrees(heading))
            drive.fieldCentric(strafe, -power, turn, heading + Math.toRadians(Drive.Companion.startingH + 90.0))
        }
    }

    override fun isFinished(): Boolean {
        return true;
    }

    companion object {
        @JvmField
        var regSpeed: Double = 0.75

        @JvmField
        var slowSpeed: Double = 0.3
    }
}