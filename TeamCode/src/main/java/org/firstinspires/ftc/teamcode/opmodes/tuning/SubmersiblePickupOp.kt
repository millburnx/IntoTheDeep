package org.firstinspires.ftc.teamcode.opmodes.tuning

import com.acmerobotics.dashboard.FtcDashboard
import com.acmerobotics.dashboard.config.Config
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry
import com.arcrobotics.ftclib.command.CommandOpMode
import com.arcrobotics.ftclib.command.ConditionalCommand
import com.arcrobotics.ftclib.command.InstantCommand
import com.arcrobotics.ftclib.gamepad.GamepadEx
import com.millburnx.utils.Vec2d
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import org.firstinspires.ftc.robotcore.external.Telemetry
import org.firstinspires.ftc.teamcode.common.commands.ArmCommand
import org.firstinspires.ftc.teamcode.common.commands.SubmersiblePickup
import org.firstinspires.ftc.teamcode.common.subsystems.Arm
import org.firstinspires.ftc.teamcode.common.subsystems.Drive
import org.firstinspires.ftc.teamcode.common.subsystems.Intake
import org.firstinspires.ftc.teamcode.common.subsystems.Lift
import org.firstinspires.ftc.teamcode.common.subsystems.misc.DeltaTime
import org.firstinspires.ftc.teamcode.common.subsystems.vision.SamplePipeline
import org.firstinspires.ftc.teamcode.common.subsystems.vision.VisionPortal
import org.firstinspires.ftc.teamcode.common.utils.GamepadSRL
import org.firstinspires.ftc.teamcode.opmodes.teleop.TeleopBeta
import kotlin.lazy

//@Disabled
@Config
@TeleOp(name = "SubmersiblePickupOp", group = "Tuning")
class SubmersiblePickupOp : CommandOpMode() {
    val samplePipeline: SamplePipeline by lazy { SamplePipeline() }
    val visionPortal: VisionPortal by lazy {
        VisionPortal(
            hardwareMap,
            "camera1",
            listOf(samplePipeline)
        )
    }
    val tel: Telemetry = FtcDashboard.getInstance().telemetry
    val drive: Drive
            by lazy {
                Drive(
                    hardwareMap,
                    org.firstinspires.ftc.teamcode.common.utils.Telemetry(),
                    FtcDashboard.getInstance()
                )
            }

    val deltaTime: DeltaTime = DeltaTime()
    val gp1: GamepadSRL by lazy {
        GamepadSRL(
            GamepadEx(gamepad1),
            TeleopBeta.maxLeftRate,
            TeleopBeta.maxRightRate,
            deltaTime
        )
    }

    val lift: Lift by lazy { Lift(hardwareMap) { 0.0 } }
    val arm: Arm by lazy { Arm(hardwareMap, tel, { 0 }) }
    val intake: Intake by lazy { Intake(hardwareMap) }

    override fun initialize() {
        visionPortal
        FtcDashboard.getInstance().startCameraStream(samplePipeline, 0.0)
        schedule(object :
            SubmersiblePickup(
                drive, lift, visionPortal.cameraSize, samplePipeline.detections::get, Vec2d(
                    offsetX, offsetY
                ), MultipleTelemetry(tel, telemetry)
            ) {
            override fun isFinished(): Boolean {
                return false
            }
        })
        gp1
    }

    override fun run() {
        super.run()
        schedule(ArmCommand(arm, armTarget))
        schedule(ConditionalCommand(
            InstantCommand({ intake.open() }),
            InstantCommand({ intake.close() }),
            { clawOpen }
        ))
    }

    companion object {
        @JvmField
        var armTarget: Int = 40

        @JvmField
        var clawOpen: Boolean = true
    }
}