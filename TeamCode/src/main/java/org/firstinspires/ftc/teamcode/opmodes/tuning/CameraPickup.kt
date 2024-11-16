package org.firstinspires.ftc.teamcode.opmodes.tuning

import com.acmerobotics.dashboard.FtcDashboard
import com.acmerobotics.dashboard.config.Config
import com.arcrobotics.ftclib.command.CommandOpMode
import com.arcrobotics.ftclib.command.ConditionalCommand
import com.arcrobotics.ftclib.command.InstantCommand
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import org.firstinspires.ftc.robotcore.external.Telemetry
import org.firstinspires.ftc.teamcode.common.commands.ArmCommand
import org.firstinspires.ftc.teamcode.common.commands.LiftCommand
import org.firstinspires.ftc.teamcode.common.commands.PickupCommand
import org.firstinspires.ftc.teamcode.common.subsystems.Arm
import org.firstinspires.ftc.teamcode.common.subsystems.Drive
import org.firstinspires.ftc.teamcode.common.subsystems.Intake
import org.firstinspires.ftc.teamcode.common.subsystems.Lift
import org.firstinspires.ftc.teamcode.common.subsystems.vision.SamplePipeline
import org.firstinspires.ftc.teamcode.common.subsystems.vision.VisionPortal

//@Disabled
@Config
@TeleOp(name = "CameraPickup", group = "Tuning")
class CameraPickup : CommandOpMode() {
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

    val lift: Lift by lazy { Lift(hardwareMap) { 0.0 } }
    val arm: Arm by lazy { Arm(hardwareMap, tel) { 0 } }
    val intake: Intake by lazy { Intake(hardwareMap) }

    override fun initialize() {
        visionPortal
        FtcDashboard.getInstance().startCameraStream(samplePipeline, 0.0)
        schedule(object :
            PickupCommand(drive, visionPortal.cameraSize, samplePipeline.detections::get) {
            override fun isFinished(): Boolean {
                return false
            }
        })
    }

    override fun run() {
        super.run()
        schedule(ArmCommand(arm, armTarget))
        schedule(LiftCommand(lift, liftTarget))
        schedule(ConditionalCommand(
            InstantCommand({ intake.open() }),
            InstantCommand({ intake.close() }),
            { clawOpen }
        ))
    }

    companion object {
        @JvmField
        var kp: Double = -0.0001

        @JvmField
        var ki: Double = 0.0

        @JvmField
        var kd: Double = 0.0

        @JvmField
        var kpRot: Double = -0.01

        @JvmField
        var kiRot: Double = 0.0

        @JvmField
        var kdRot: Double = 0.0

        @JvmField
        var strafeMulti: Double = 1.8

        @JvmField
        var offsetX: Double = 0.0 // -1.0 to 1.0

        @JvmField
        var offsetY: Double = 0.0 // -1.0 to 1.0

        @JvmField
        var armTarget: Int = 70

        @JvmField
        var armThres: Int = 30

        @JvmField
        var liftTarget: Double = Lift.base

        @JvmField
        var maxSpeed = 0.3

        @JvmField
        var maxRotation = 0.1

        @JvmField
        var clawOpen: Boolean = true
    }
}