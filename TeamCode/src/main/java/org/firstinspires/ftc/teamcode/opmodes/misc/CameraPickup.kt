package org.firstinspires.ftc.teamcode.opmodes.misc

import com.acmerobotics.dashboard.FtcDashboard
import com.acmerobotics.dashboard.config.Config
import com.arcrobotics.ftclib.command.CommandOpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import org.firstinspires.ftc.robotcore.external.Telemetry
import org.firstinspires.ftc.teamcode.common.commands.ArmCommand
import org.firstinspires.ftc.teamcode.common.subsystems.Arm
import org.firstinspires.ftc.teamcode.common.subsystems.Drive
import org.firstinspires.ftc.teamcode.common.subsystems.vision.SampleColor
import org.firstinspires.ftc.teamcode.common.subsystems.vision.SamplePipeline
import org.firstinspires.ftc.teamcode.common.subsystems.vision.VisionPortal

//@Disabled
@Config
@TeleOp(name = "CameraPickup", group = "Misc")
class CameraPickup : CommandOpMode() {
    val samplePipeline: SamplePipeline by lazy { SamplePipeline() }
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
    val arm: Arm by lazy { Arm(hardwareMap, tel) { 0 } }

    override fun initialize() {
        visionPortal
        FtcDashboard.getInstance().startCameraStream(samplePipeline, 0.0)
    }

    override fun run() {
        super.run()
        schedule(ArmCommand(arm, 0))

        val samples = samplePipeline.detections.get()

        samples.forEachIndexed { i, sample ->
            tel.addData("sample $i", sample.toString())
        }

        tel.addData("TOTAL SAMPLES: ", "${samples.size}")
        tel.addData("RED SAMPLES: ", "{${samples.filter { sample -> sample.color == SampleColor.RED }.size}}")
        tel.addData("YELLOW SAMPLES: ", "{${samples.filter { sample -> sample.color == SampleColor.YELLOW }.size}}")
        tel.addData("BLUE SAMPLES: ", "{${samples.filter { sample -> sample.color == SampleColor.BLUE }.size}}")

        tel.addData("FPS", visionPortal.visionPortal.fps)

        // largest sample on screen
        val targetSample = samples.maxByOrNull { sample -> sample.boundingBox.area }
        val targetSampleText =
            if (targetSample != null) "p(${targetSample.pos}), s (${targetSample.boundingBox.area}), c(${targetSample.color})" else "none"
        tel.addData("targetSample", targetSampleText)

        if (targetSample != null) {
            val cameraCenter = (visionPortal.cameraSize / 2).flip()
            val sampleCenter = targetSample.pos.flip()

            val diff = sampleCenter - cameraCenter
            drive.robotCentric(diff.y * speed, diff.x * speed, 0.0)
            tel.addData("diff", diff)
        }

        tel.update()
    }

    companion object {
        @JvmField
        var speed: Double = 0.0
    }
}