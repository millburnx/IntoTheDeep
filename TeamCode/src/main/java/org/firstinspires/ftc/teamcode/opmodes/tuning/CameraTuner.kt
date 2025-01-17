package org.firstinspires.ftc.teamcode.opmodes.tuning

import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import org.firstinspires.ftc.teamcode.common.Robot
import org.firstinspires.ftc.teamcode.common.processors.SampleDetector
import org.firstinspires.ftc.teamcode.common.subsystems.vision.Vision
import org.firstinspires.ftc.teamcode.common.utils.OpMode
import org.firstinspires.ftc.vision.VisionProcessor

open class CameraRobot(
    opMode: OpMode,
) : Robot(opMode) {
    open val camera = Vision(this)
    override val additionalSubsystems = listOf(camera)

    override fun init() {
        imu.resetYaw()
        super.init()
    }
}

@TeleOp(name = "Camera Tuner")
class CameraTuner : OpMode() {
    override val robot by lazy { CameraRobot(this) }

    override fun exec() {
        telemetry.addData("state", robot.camera.visionPortal.cameraState)
        telemetry.addData("FPS", robot.camera.visionPortal.fps)
    }
}

class SampleVision(
    robot: Robot,
) : Vision(robot) {
    val sampleDetector = SampleDetector()

    override val processors =
        listOf<VisionProcessor>(
            sampleDetector,
        )
}

class SampleCameraRobot(
    opMode: OpMode,
) : CameraRobot(opMode) {
    override val camera = SampleVision(this)
}

@TeleOp(name = "Sample Dectector Tuner")
class SampleDectectionTuner : OpMode() {
    override val robot by lazy { SampleCameraRobot(this) }

    override fun exec() {
        telemetry.addData(
            "Red Samples",
            robot.camera.sampleDetector.redSamples
                .get()
                .size,
        )
        telemetry.addData(
            "Yellow Samples",
            robot.camera.sampleDetector.yellowSamples
                .get()
                .size,
        )
        telemetry.addData(
            "Blue Samples",
            robot.camera.sampleDetector.blueSamples
                .get()
                .size,
        )
        telemetry.addData("state", robot.camera.visionPortal.cameraState)
        telemetry.addData("FPS", robot.camera.visionPortal.fps)
    }
}
