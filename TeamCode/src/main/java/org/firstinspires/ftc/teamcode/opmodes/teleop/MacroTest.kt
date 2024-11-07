package org.firstinspires.ftc.teamcode.opmodes.teleop

import com.acmerobotics.dashboard.FtcDashboard
import com.acmerobotics.dashboard.config.Config
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry
import com.arcrobotics.ftclib.command.CommandOpMode
import com.arcrobotics.ftclib.command.InstantCommand
import com.arcrobotics.ftclib.command.RunCommand
import com.arcrobotics.ftclib.gamepad.GamepadEx
import com.arcrobotics.ftclib.gamepad.GamepadKeys
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import org.firstinspires.ftc.teamcode.common.commands.DriveRobotCommand
import org.firstinspires.ftc.teamcode.common.commands.SpecimenDown
import org.firstinspires.ftc.teamcode.common.commands.SpecimenDown2
import org.firstinspires.ftc.teamcode.common.commands.SpecimenUp
import org.firstinspires.ftc.teamcode.common.subsystems.Arm
import org.firstinspires.ftc.teamcode.common.subsystems.Drive
import org.firstinspires.ftc.teamcode.common.subsystems.Intake
import org.firstinspires.ftc.teamcode.common.subsystems.Lift
import org.firstinspires.ftc.teamcode.common.utils.Telemetry

@Config
@TeleOp(name = "Macro Test")
class MacroTest : CommandOpMode() {
    val tel: Telemetry = Telemetry()
    val dash: FtcDashboard = FtcDashboard.getInstance()
    val telem = MultipleTelemetry(telemetry, dash.telemetry)

    val drive: Drive by lazy {
        Drive(hardwareMap, tel, dash)
    }
    val gamepad1Ex: GamepadEx by lazy {
        GamepadEx(gamepad1)
    }
    val gamepad2Ex: GamepadEx by lazy {
        GamepadEx(gamepad2)
    }
    val arm: Arm by lazy {
        Arm(hardwareMap, telemetry, lift.lift::getCurrentPosition)
    }
    val lift: Lift by lazy {
        Lift(hardwareMap)
    }
    val intake: Intake by lazy {
        Intake(hardwareMap)
    }

    override fun initialize() {
        drive.defaultCommand =
            RunCommand({
                schedule(
                    DriveRobotCommand(
                        drive,
                        gamepad1Ex,
                        telemetry,
                        { true },
                        { true }
                    )
                )
            }, drive)
        lift.armAngle = arm::angle
    }

    override fun run() {
        super.run()

        gamepad1Ex.getGamepadButton(GamepadKeys.Button.LEFT_BUMPER).whenPressed(InstantCommand({ intake.close() }))
        gamepad1Ex.getGamepadButton(GamepadKeys.Button.RIGHT_BUMPER).whenPressed(InstantCommand({ intake.open() }))

        gamepad1Ex.getGamepadButton(GamepadKeys.Button.Y).whenPressed(SpecimenUp(arm, lift, intake))
        gamepad1Ex.getGamepadButton(GamepadKeys.Button.X).whenPressed(SpecimenDown(arm, lift, intake))
        gamepad1Ex.getGamepadButton(GamepadKeys.Button.A).whenPressed(SpecimenDown2(arm, lift, intake))

        telem.addData("arm pos: ", arm.position)
        telem.addData("arm angle: ", arm.angle)
        telem.addData("arm target: ", arm.target)
        telem.addData("arm power: ", -arm.rightRotate.power)

        telem.addData("lift pos: ", lift.position)
        telem.addData("lift target; ", lift.target)
        telem.addData("lift power: ", lift.lift.power)

        telem.addData("intake open: ", intake.open)

        val pose = drive.pose
        telem.addData("x", pose.x)
        telem.addData("y", pose.y)
        telem.addData("heading", Math.toDegrees(pose.heading))

        telem.update()
    }

    companion object {
        @JvmField
        var fieldCentric: Boolean = false

        @JvmField
        var flipY: Boolean = false

        @JvmField
        var manualArm = 0.01

        @JvmField
        var slowManualArm = 0.005

        @JvmField
        var manualLift = 70.0

        @JvmField
        var slowManualLift = 37.5

        @JvmField
        var d1Cubic = true

        @JvmField
        var d1CubicAll = true

        @JvmField
        var d2Cubic = true
    }
}
