package org.firstinspires.ftc.teamcode.opmodes.teleop

import com.acmerobotics.dashboard.FtcDashboard
import com.acmerobotics.dashboard.config.Config
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry
import com.arcrobotics.ftclib.command.CommandOpMode
import com.arcrobotics.ftclib.gamepad.GamepadEx
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import org.firstinspires.ftc.robotcore.external.Telemetry
import org.firstinspires.ftc.teamcode.common.commands.ArmCommand
import org.firstinspires.ftc.teamcode.common.commands.LiftCommand
import org.firstinspires.ftc.teamcode.common.subsystems.Arm
import org.firstinspires.ftc.teamcode.common.subsystems.Drive
import org.firstinspires.ftc.teamcode.common.subsystems.Lift
import org.firstinspires.ftc.teamcode.common.subsystems.misc.DeltaTime
import org.firstinspires.ftc.teamcode.common.utils.GamepadSRL

@Config
@TeleOp(name = "Teleop Beta")
class TeleopBeta : CommandOpMode() {
    val tel: Telemetry = MultipleTelemetry(telemetry, FtcDashboard.getInstance().telemetry)
    val telUtil: org.firstinspires.ftc.teamcode.common.utils.Telemetry =
        org.firstinspires.ftc.teamcode.common.utils.Telemetry()

    val deltaTime: DeltaTime = DeltaTime()

    val gp1: GamepadSRL by lazy {
        GamepadSRL(GamepadEx(gamepad1), maxLeftRate, maxRightRate, deltaTime)
    }
    val drive: Drive by lazy {
        Drive(hardwareMap, telUtil, FtcDashboard.getInstance())
    }
    val lift: Lift by lazy {
        Lift(hardwareMap)
    }
    val arm: Arm by lazy {
        Arm(hardwareMap, telemetry, lift.lift::getCurrentPosition)
    }

    override fun initialize() {
        lift.armAngle = arm::angle
    }

    override fun run() {
        super.run()
        schedule(ArmCommand(arm, armTarget))
        schedule(LiftCommand(lift, liftTarget))

        if (!fieldCentric) {
            drive.robotCentric(gp1.leftStick.y, gp1.leftStick.x * 1.1, gp1.rightStick.x)
        } else {
            val heading = drive.imuHeading
            tel.addData("heading (imu)", Math.toDegrees(heading))
            drive.fieldCentric(gp1.leftStick.x, -gp1.leftStick.y, gp1.rightStick.x, heading + Math.toRadians(90.0))
        }

        tel.addData("leftStick: ", gp1.leftStick)
        tel.addData("rightStick: ", gp1.rightStick)
        tel.addData("arm pos: ", arm.position)
        tel.addData("lift pos: ", lift.position)
        tel.addData("delta time: ", deltaTime.deltaTime)
        tel.update()
    }

    companion object {
        @JvmField
        var maxLeftRate = 8.0

        @JvmField
        var maxRightRate = 8.0

        @JvmField
        var fieldCentric = true

        @JvmField
        var armTarget: Int = 15

        @JvmField
        var liftTarget: Int = 30
    }
}