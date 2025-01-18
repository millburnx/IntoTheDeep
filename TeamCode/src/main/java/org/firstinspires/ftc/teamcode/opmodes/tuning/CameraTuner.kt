package org.firstinspires.ftc.teamcode.opmodes.tuning

import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import org.firstinspires.ftc.teamcode.common.Robot
import org.firstinspires.ftc.teamcode.common.subsystems.vision.SampleVision
import org.firstinspires.ftc.teamcode.common.subsystems.vision.Vision
import org.firstinspires.ftc.teamcode.common.utils.OpMode

open class CameraRobot(
    opMode: OpMode,
) : Robot(opMode) {
    open val camera by lazy { Vision(this) }
    override val additionalSubsystems = listOf(camera)

    override fun init() {
        imu.resetYaw()
        super.init()
    }
}

class SampleCameraRobot(
    opMode: OpMode,
) : CameraRobot(opMode) {
    override val camera by lazy { SampleVision(this) }
}

@TeleOp(name = "Camera Tuner")
class CameraTuner : OpMode() {
    override val robot by lazy { CameraRobot(this) }

    override fun exec() {
        robot.telemetry.addData("state 1", robot.camera.camera1.cameraState)
        robot.telemetry.addData("FPS 1", robot.camera.camera1.fps)

        robot.telemetry.addData("state 2", robot.camera.camera2.cameraState)
        robot.telemetry.addData("FPS 2", robot.camera.camera2.fps)
    }
}

@TeleOp(name = "Sample Dectector Tuner")
class SampleDectectionTuner : OpMode() {
    override val robot by lazy { SampleCameraRobot(this) }

    override fun exec() {
//        robot.telemetry.addData(
//            "Red Samples 1",
//            robot.camera.sampleDetector1.redSamples
//                .get()
//                .size,
//        )
//        robot.telemetry.addData(
//            "Yellow Samples 1",
//            robot.camera.sampleDetector1.yellowSamples
//                .get()
//                .size,
//        )
//        robot.telemetry.addData(
//            "Blue Samples 1",
//            robot.camera.sampleDetector1.blueSamples
//                .get()
//                .size,
//        )
//
//        robot.telemetry.addData(
//            "Red Samples 2",
//            robot.camera.sampleDetector2.redSamples
//                .get()
//                .size,
//        )
//        robot.telemetry.addData(
//            "Yellow Samples 2",
//            robot.camera.sampleDetector2.yellowSamples
//                .get()
//                .size,
//        )
//        robot.telemetry.addData(
//            "Blue Samples 2",
//            robot.camera.sampleDetector2.blueSamples
//                .get()
//                .size,
//        )

        robot.telemetry.addData("state 1", robot.camera.camera1.cameraState)
        robot.telemetry.addData("FPS 1", robot.camera.camera1.fps)

        robot.telemetry.addData("state 2", robot.camera.camera2.cameraState)
        robot.telemetry.addData("FPS 2", robot.camera.camera2.fps)
    }
}
