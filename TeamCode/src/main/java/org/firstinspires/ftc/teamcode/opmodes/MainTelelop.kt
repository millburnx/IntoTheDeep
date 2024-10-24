package org.firstinspires.ftc.teamcode.opmodes

import com.acmerobotics.dashboard.FtcDashboard
import com.acmerobotics.dashboard.config.Config
import com.arcrobotics.ftclib.command.CommandOpMode
import com.arcrobotics.ftclib.command.InstantCommand
import com.arcrobotics.ftclib.command.ParallelCommandGroup
import com.arcrobotics.ftclib.gamepad.GamepadEx
import com.arcrobotics.ftclib.gamepad.GamepadKeys
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import org.firstinspires.ftc.teamcode.common.commands.ArmCommand
import org.firstinspires.ftc.teamcode.common.commands.DriveRobotCommand
import org.firstinspires.ftc.teamcode.common.commands.LiftCommand
import org.firstinspires.ftc.teamcode.common.subsystems.Arm
import org.firstinspires.ftc.teamcode.common.subsystems.Drive
import org.firstinspires.ftc.teamcode.common.subsystems.Intake
import org.firstinspires.ftc.teamcode.common.subsystems.Lift
import org.firstinspires.ftc.teamcode.common.utils.Telemetry

@Config
@TeleOp(name = "MainTeleOp")
class MainTelelop : CommandOpMode() {
    val tel: Telemetry = Telemetry()
    val dash: FtcDashboard = FtcDashboard.getInstance()

    val drive: Drive by lazy {
        Drive(hardwareMap, tel, dash, -1.0)
    }
    val gamepad1Ex: GamepadEx by lazy {
        GamepadEx(gamepad1)
    }
    val gamepad2Ex: GamepadEx by lazy {
        GamepadEx(gamepad2)
    }
    val lift: Lift by lazy {
        Lift(hardwareMap)
    }
    val arm: Arm by lazy {
        Arm(hardwareMap, telemetry, lift.lift::getCurrentPosition)
    }
    val intake: Intake by lazy {
        Intake(hardwareMap)
    }

    override fun initialize() {
        drive.defaultCommand = DriveRobotCommand(drive, gamepad1Ex, telemetry)
    }

    override fun run() {
        super.run()

        gamepad1Ex.getGamepadButton(GamepadKeys.Button.B).whenPressed(InstantCommand(drive.imu::resetYaw))

        gamepad1Ex.getGamepadButton(GamepadKeys.Button.DPAD_DOWN).whenPressed(ArmCommand(arm, Arm.base))
        gamepad1Ex.getGamepadButton(GamepadKeys.Button.DPAD_UP).whenPressed(ArmCommand(arm, Arm.lowBasket))
        gamepad1Ex.getGamepadButton(GamepadKeys.Button.DPAD_RIGHT).whenPressed(ArmCommand(arm, Arm.pickup))
        gamepad1Ex.getGamepadButton(GamepadKeys.Button.DPAD_LEFT).whenPressed(InstantCommand(arm::off))

        gamepad1Ex.getGamepadButton(GamepadKeys.Button.A).whenPressed(LiftCommand(lift, Lift.base)) // cross
        gamepad1Ex.getGamepadButton(GamepadKeys.Button.Y).whenPressed(LiftCommand(lift, Lift.lowBasket)) // triangle
        gamepad1Ex.getGamepadButton(GamepadKeys.Button.X).whenPressed(LiftCommand(lift, Lift.pickup)) // square

        gamepad2Ex.getGamepadButton(GamepadKeys.Button.A).whenPressed(
            ParallelCommandGroup(
                LiftCommand(lift, Lift.base),
                ArmCommand(arm, Arm.base)
            )
        )// cross
        gamepad2Ex.getGamepadButton(GamepadKeys.Button.Y).whenPressed(
            ParallelCommandGroup(
                LiftCommand(lift, Lift.lowBasket),
                ArmCommand(arm, Arm.lowBasket)
            )
        ) // triangle
        gamepad2Ex.getGamepadButton(GamepadKeys.Button.X).whenPressed(
            ParallelCommandGroup(
                LiftCommand(lift, Lift.pickup),
                ArmCommand(arm, Arm.pickup)
            )
        ) // square

        gamepad1Ex.getGamepadButton(GamepadKeys.Button.LEFT_BUMPER).whileHeld(InstantCommand(intake::intake))
        gamepad1Ex.getGamepadButton(GamepadKeys.Button.RIGHT_BUMPER).whileHeld(InstantCommand(intake::outtake))
        gamepad1Ex.getGamepadButton(GamepadKeys.Button.BACK).whileHeld(InstantCommand(intake::stop))

        telemetry.update()
    }

    companion object {
        var fieldCentric: Boolean = false
        var flipY: Boolean = false
    }
}
