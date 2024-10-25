package org.firstinspires.ftc.teamcode.opmodes

import com.acmerobotics.dashboard.FtcDashboard
import com.acmerobotics.dashboard.config.Config
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry
import com.arcrobotics.ftclib.command.CommandOpMode
import com.arcrobotics.ftclib.command.InstantCommand
import com.arcrobotics.ftclib.command.SequentialCommandGroup
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
    val telem = MultipleTelemetry(telemetry, dash.telemetry)

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

        intake.defaultCommand = InstantCommand(intake::stop, intake)
    }

    override fun run() {
        super.run()

        gamepad1Ex.getGamepadButton(GamepadKeys.Button.B).whenPressed(InstantCommand(drive.imu::resetYaw))

        gamepad1Ex.getGamepadButton(GamepadKeys.Button.DPAD_UP).whenPressed(ArmCommand(arm, Arm.lowBasket))
        gamepad1Ex.getGamepadButton(GamepadKeys.Button.DPAD_RIGHT).whenPressed(ArmCommand(arm, Arm.base))
        gamepad1Ex.getGamepadButton(GamepadKeys.Button.DPAD_DOWN).whenPressed(ArmCommand(arm, Arm.pickup))
        gamepad1Ex.getGamepadButton(GamepadKeys.Button.DPAD_LEFT).whenPressed(InstantCommand(arm::off))

        gamepad1Ex.getGamepadButton(GamepadKeys.Button.Y).whenPressed(LiftCommand(lift, Lift.lowBasket)) // triangle
        gamepad1Ex.getGamepadButton(GamepadKeys.Button.X).whenPressed(LiftCommand(lift, Lift.base)) // cross
        gamepad1Ex.getGamepadButton(GamepadKeys.Button.A).whenPressed(LiftCommand(lift, Lift.pickup)) // square

        gamepad2Ex.getGamepadButton(GamepadKeys.Button.Y).whenPressed(
            SequentialCommandGroup(
                LiftCommand(lift, Lift.base),
                ArmCommand(arm, Arm.lowBasket),
                LiftCommand(lift, Lift.lowBasket),
            )
        ) // triangle
        // need the change the order of return to base based on the previous state (this really would be better as fsm)
        // because from pickup, you lift arm then retract, from basket you retract then lower arm
        gamepad2Ex.getGamepadButton(GamepadKeys.Button.X).whenPressed(
            SequentialCommandGroup(
                ArmCommand(arm, Arm.base),
                LiftCommand(lift, Lift.base),
            )
        )// cross
        gamepad2Ex.getGamepadButton(GamepadKeys.Button.A).whenPressed(
            SequentialCommandGroup(
                LiftCommand(lift, Lift.pickup),
                ArmCommand(arm, Arm.pickup)
            )
        ) // square

        if (gamepad2.left_trigger > 0.2) {
            lift.target -= 1
        } else if (gamepad2.right_trigger > 0.2) {
            lift.target += 1
        }

        gamepad1Ex.getGamepadButton(GamepadKeys.Button.LEFT_BUMPER).whileHeld(InstantCommand(intake::intake))
        gamepad1Ex.getGamepadButton(GamepadKeys.Button.RIGHT_BUMPER).whileHeld(InstantCommand(intake::outtake))

        telem.addData("arm pos: ", arm.position)
        telem.addData("arm angle: ", arm.angle)
        telem.addData("arm target: ", arm.target)

        telem.addData("lift pos: ", lift.position)
        telem.addData("lift target; ", lift.target)

        val pose = drive.pose
        telem.addData("x", pose.x)
        telem.addData("y", pose.y)
        telem.addData("heading", Math.toDegrees(pose.heading))
        telem.update()
    }

    companion object {
        var fieldCentric: Boolean = false
        var flipY: Boolean = false
    }
}
