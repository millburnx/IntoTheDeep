package org.firstinspires.ftc.teamcode.opmodes.teleop

import com.acmerobotics.dashboard.FtcDashboard
import com.acmerobotics.dashboard.config.Config
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry
import com.arcrobotics.ftclib.command.CommandOpMode
import com.arcrobotics.ftclib.command.ConditionalCommand
import com.arcrobotics.ftclib.command.InstantCommand
import com.arcrobotics.ftclib.command.RunCommand
import com.arcrobotics.ftclib.command.SequentialCommandGroup
import com.arcrobotics.ftclib.gamepad.GamepadEx
import com.arcrobotics.ftclib.gamepad.GamepadKeys
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import com.qualcomm.robotcore.hardware.DcMotor
import org.firstinspires.ftc.teamcode.common.commands.ArmCommand
import org.firstinspires.ftc.teamcode.common.commands.DriveRobotCommand
import org.firstinspires.ftc.teamcode.common.commands.LiftCommand
import org.firstinspires.ftc.teamcode.common.subsystems.Arm
import org.firstinspires.ftc.teamcode.common.subsystems.Drive
import org.firstinspires.ftc.teamcode.common.subsystems.Intake
import org.firstinspires.ftc.teamcode.common.subsystems.Lift
import org.firstinspires.ftc.teamcode.common.utils.Telemetry
import kotlin.math.abs

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
    var slowDriveMode: Boolean = false;
    var slowMechMode: Boolean = false;

    override fun initialize() {
        drive.defaultCommand =
            RunCommand({ schedule(DriveRobotCommand(drive, gamepad1Ex, telemetry, { slowDriveMode })) }, drive)

    }

    override fun run() {
        super.run()

        if (abs(gamepad2Ex.leftX) > 0.1 || abs(gamepad2Ex.leftY) > 0.1 || abs(gamepad2Ex.rightX) > 0.1) {
            schedule(DriveRobotCommand(drive, gamepad2Ex, telemetry, { true }))
        }

        gamepad1Ex.getGamepadButton(GamepadKeys.Button.RIGHT_BUMPER).whenPressed(
            InstantCommand({ slowDriveMode = true })
        ).whenReleased(
            InstantCommand({ slowDriveMode = false })
        )

        gamepad1Ex.getGamepadButton(GamepadKeys.Button.DPAD_RIGHT)
            .whenPressed(InstantCommand(drive.imu::resetYaw, drive))


        gamepad2Ex.getGamepadButton(GamepadKeys.Button.LEFT_BUMPER).whenPressed(
            InstantCommand({ slowMechMode = true })
        ).whenReleased(
            InstantCommand({ slowMechMode = false })
        )


        gamepad2Ex.getGamepadButton(GamepadKeys.Button.DPAD_LEFT).whileHeld(InstantCommand(intake::close, intake))
        gamepad2Ex.getGamepadButton(GamepadKeys.Button.DPAD_RIGHT).whileHeld(InstantCommand(intake::open, intake))
        gamepad2Ex.getGamepadButton(GamepadKeys.Button.RIGHT_BUMPER).whenPressed(
            InstantCommand(
                intake::toggle, intake
            ),
        )

        gamepad1Ex.getGamepadButton(GamepadKeys.Button.B).whenPressed(InstantCommand({
            arm.leftRotate.mode = DcMotor.RunMode.RESET_ENCODERS
            arm.rightRotate.mode = DcMotor.RunMode.RESET_ENCODERS
            lift.lift.mode = DcMotor.RunMode.RESET_ENCODERS
            arm.leftRotate.mode = DcMotor.RunMode.RUN_WITHOUT_ENCODER
            arm.rightRotate.mode = DcMotor.RunMode.RUN_WITHOUT_ENCODER
            lift.lift.mode = DcMotor.RunMode.RUN_WITHOUT_ENCODER
        }))

//        gamepad2Ex.getGamepadButton(GamepadKeys.Button.DPAD_LEFT).whenPressed(InstantCommand(arm::off))

        gamepad2Ex.getGamepadButton(GamepadKeys.Button.Y).whenPressed(
            SequentialCommandGroup(
                // TODO: CHECK IF ARM IS AT PICKUP OR BASE, IF AT PICKUP WE CAN GO TO BASE FIRST OR REJECT
                ArmCommand(arm, Arm.lowBasket),
                LiftCommand(lift, Lift.lowBasket),
            )
        ) // triangle
        gamepad2Ex.getGamepadButton(GamepadKeys.Button.X).whenPressed(
            ConditionalCommand(
                SequentialCommandGroup(
                    ArmCommand(arm, Arm.base).withTimeout(3000),
                    LiftCommand(lift, Lift.base),
//                    InstantCommand(arm::off), // TODO: UNCOMMENT TO STOP IDLE POWER
                ),
                SequentialCommandGroup(
                    LiftCommand(lift, Lift.base),
                    ArmCommand(arm, Arm.base),
//                    InstantCommand(arm::off), // TODO: UNCOMMENT IF STOP IDLE POWER
                )
            ) { arm.position < Arm.base + (Arm.lowBasket - Arm.base) / 2 }
        )// square
        gamepad2Ex.getGamepadButton(GamepadKeys.Button.A).whenPressed(
            SequentialCommandGroup(
                LiftCommand(lift, Lift.pickup),
                ArmCommand(arm, Arm.pickup)
            )
        ) // cross
        gamepad2Ex.getGamepadButton(GamepadKeys.Button.B).whenPressed(
            SequentialCommandGroup(
                // TODO: CHECK IF ARM IS AT PICKUP OR BASE, IF AT PICKUP WE CAN GO TO BASE FIRST OR REJECT
                ArmCommand(arm, Arm.highBasket),
                LiftCommand(lift, Lift.highBasket)
            )
        ) // circle

        if (gamepad2.left_trigger > 0.1) {
//            lift.target -= gamepad2.left_trigger
            schedule(
                InstantCommand(
                    { lift.target -= gamepad2.left_trigger * if (slowMechMode) slowManualLift else manualLift },
                    lift
                )
            )
        } else if (gamepad2.right_trigger > 0.1) {
//            lift.target += gamepad2.right_trigger
            schedule(
                InstantCommand(
                    { lift.target += gamepad2.right_trigger * if (slowMechMode) slowManualLift else manualLift },
                    lift
                )
            )
        }

        gamepad2Ex.getGamepadButton(GamepadKeys.Button.DPAD_UP)
            .whileHeld(
                InstantCommand(
                    { arm.on(); arm.target += if (slowMechMode) slowManualArm else manualArm },
                    arm
                )
            )
        gamepad2Ex.getGamepadButton(GamepadKeys.Button.DPAD_DOWN)
            .whileHeld(
                InstantCommand(
                    { arm.on(); arm.target -= if (slowMechMode) slowManualArm else manualArm },
                    arm
                )
            )

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
        var manualLift = 60.0

        @JvmField
        var slowManualLift = 37.5
    }
}
