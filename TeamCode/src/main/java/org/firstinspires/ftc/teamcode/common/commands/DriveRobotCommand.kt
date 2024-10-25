package org.firstinspires.ftc.teamcode.common.commands

import com.arcrobotics.ftclib.command.CommandBase
import com.arcrobotics.ftclib.gamepad.GamepadEx
import org.firstinspires.ftc.robotcore.external.Telemetry
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit
import org.firstinspires.ftc.teamcode.common.subsystems.Drive
import org.firstinspires.ftc.teamcode.opmodes.MainTelelop.Companion.fieldCentric
import org.firstinspires.ftc.teamcode.opmodes.MainTelelop.Companion.flipY


class DriveRobotCommand(val drive: Drive, val gamepad: GamepadEx, val telemetry: Telemetry) : CommandBase() {
    init {
        addRequirements(drive)
    }

    override fun execute() {
        val power = gamepad.leftY * if (flipY) -1.0 else 1.0
        val strafe = gamepad.leftX * 1.1
        val turn = gamepad.rightX

        if (!fieldCentric) {
            drive.robotCentric(power, strafe, turn)
        } else {
            val heading = drive.imu.robotYawPitchRollAngles.getYaw(AngleUnit.RADIANS)
            telemetry.addData("heading", Math.toDegrees(heading))
            drive.fieldCentric(power, strafe, turn, heading + Drive.Companion.startingH)
        }
    }
}