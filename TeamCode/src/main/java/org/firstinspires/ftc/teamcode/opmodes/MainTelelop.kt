package org.firstinspires.ftc.teamcode.opmodes

import com.acmerobotics.dashboard.FtcDashboard
import com.acmerobotics.dashboard.config.Config
import com.arcrobotics.ftclib.command.CommandOpMode
import com.arcrobotics.ftclib.gamepad.GamepadEx
import com.arcrobotics.ftclib.gamepad.GamepadKeys
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import org.firstinspires.ftc.teamcode.common.commands.ArmCommand
import org.firstinspires.ftc.teamcode.common.commands.DriveRobotCommand
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

        if (gamepad1.b) {
            println("RESETTING IMU")
            drive.imu.resetYaw()
        }

        gamepad1Ex.getGamepadButton(GamepadKeys.Button.DPAD_DOWN).whenPressed(ArmCommand(arm, Arm.base))
        gamepad1Ex.getGamepadButton(GamepadKeys.Button.DPAD_UP).whenPressed(ArmCommand(arm, Arm.lowBasket))
        gamepad1Ex.getGamepadButton(GamepadKeys.Button.DPAD_RIGHT).whenPressed(ArmCommand(arm, Arm.floor))

        // lift
        if (gamepad1.cross) {
            lift.target = Lift.base
        } else if (gamepad1.triangle) {
            lift.target = Lift.lowBasket
        } else if (gamepad1.square) {
            lift.target = Lift.pickup
        }

        // intake
        if (gamepad1.left_bumper) {
            intake.intake()
        } else if (gamepad1.right_bumper) {
            intake.outtake()
        } else {
            intake.stop()
        }

        lift.run()
        telemetry.update()
    }

    companion object {
        var fieldCentric: Boolean = false
        var flipY: Boolean = false
    }
}
