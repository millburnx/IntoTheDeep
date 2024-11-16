package org.firstinspires.ftc.teamcode.opmodes.tuning

import com.ThermalEquilibrium.homeostasis.Controllers.Feedback.PIDEx
import com.ThermalEquilibrium.homeostasis.Parameters.PIDCoefficientsEx
import com.acmerobotics.dashboard.FtcDashboard
import com.acmerobotics.dashboard.config.Config
import com.arcrobotics.ftclib.command.CommandOpMode
import com.arcrobotics.ftclib.command.ConditionalCommand
import com.arcrobotics.ftclib.command.InstantCommand
import com.millburnx.utils.Vec2d
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import org.firstinspires.ftc.robotcore.external.Telemetry
import org.firstinspires.ftc.teamcode.common.commands.ArmCommand
import org.firstinspires.ftc.teamcode.common.subsystems.Arm
import org.firstinspires.ftc.teamcode.common.subsystems.Drive
import org.firstinspires.ftc.teamcode.common.subsystems.Intake
import org.firstinspires.ftc.teamcode.common.subsystems.Lift
import org.firstinspires.ftc.teamcode.common.subsystems.vision.ClipPipeline
import org.firstinspires.ftc.teamcode.common.subsystems.vision.VisionPortal

//@Disabled
@Config
@TeleOp(name = "CameraPickup - Clip", group = "Tuning")
class CameraPickupClip : CommandOpMode() {
    val samplePipeline: ClipPipeline by lazy { ClipPipeline() }
    val visionPortal: VisionPortal by lazy { VisionPortal(hardwareMap, "camera1", listOf(samplePipeline)) }
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
    val arm: Arm by lazy { Arm(hardwareMap, tel, { 0 }) }
    val xPID: PIDEx = PIDEx(PIDCoefficientsEx(kp, ki, kd, kSum, kStab, 0.3))
    val yPID: PIDEx = PIDEx(PIDCoefficientsEx(kp, ki, kd, kSum, kStab, 0.3))
    val rPID: PIDEx = PIDEx(PIDCoefficientsEx(kpRot, kiRot, kdRot, 0.25 / kiRot, 0.1, 0.3))
    val intake: Intake by lazy { Intake(hardwareMap) }

    override fun initialize() {
        visionPortal
        FtcDashboard.getInstance().startCameraStream(samplePipeline, 0.0)
    }

    override fun run() {
        super.run()
        schedule(ArmCommand(arm, armTarget))
        schedule(ConditionalCommand(
            InstantCommand({ intake.open() }),
            InstantCommand({ intake.close() }),
            { clawOpen }
        ))

        val samples = samplePipeline.detections.get()

        samples.forEachIndexed { i, sample ->
            tel.addData("sample $i", sample.toString())
        }

        tel.addData("TOTAL Clips: ", "${samples.size}")
        tel.addData("FPS", visionPortal.visionPortal.fps)

        // largest sample on screen
        val targetSample = samples.maxByOrNull { sample -> sample.boundingBox.area }
        val targetSampleText =
            if (targetSample != null) "p(${targetSample.pos}), s (${targetSample.boundingBox.area})" else "none"
        tel.addData("targetSample", targetSampleText)

        if (targetSample != null && arm.target > armThres) {
            val offset = Vec2d(offsetX, offsetY)
            val cameraCenter = (visionPortal.cameraSize / 2).flip()
            val targetCenter = cameraCenter * (offset + 1)
            val sampleCenter = targetSample.pos.flip()

            val diff = targetCenter - sampleCenter
            val diffH = targetSample.angle

            val xPower = xPID.calculate(0.0, diff.x).coerceIn(-maxSpeed, maxSpeed) * strafeMulti
            val yPower = yPID.calculate(0.0, diff.y).coerceIn(-maxSpeed, maxSpeed)
            val rPower = rPID.calculate(0.0, diffH).coerceIn(-maxRotation, maxRotation)

            drive.robotCentric(yPower, xPower, rPower)
            tel.addData("diff", diff)
            tel.addData("powerX", xPower)
            tel.addData("powerY", yPower)
            tel.addData("diffH", diffH)
            tel.addData("powerH", rPower)
        } else {
            drive.robotCentric(0.0, 0.0, 0.0)
        }

        tel.update()
    }

    companion object {
        @JvmField
        var kp: Double = 0.001

        @JvmField
        var ki: Double = 0.01

        @JvmField
        var kd: Double = 0.0

        @JvmField
        var kStab: Double = 0.15

        @JvmField
        var kSum: Double = 0.4

        @JvmField
        var kpRot: Double = 0.01

        @JvmField
        var kiRot: Double = 0.0

        @JvmField
        var kdRot: Double = 0.0

        @JvmField
        var strafeMulti: Double = 1.35

        @JvmField
        var offsetX: Double = 0.0 // -1.0 to 1.0

        @JvmField
        var offsetY: Double = 0.0 // -1.0 to 1.0

        @JvmField
        var armTarget: Int = 70

        @JvmField
        var armThres: Int = 30

        @JvmField
        var maxSpeed = 0.2

        @JvmField
        var maxRotation = 0.2

        @JvmField
        var clawOpen: Boolean = true
    }
}