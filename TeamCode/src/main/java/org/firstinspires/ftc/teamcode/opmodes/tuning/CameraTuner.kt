package org.firstinspires.ftc.teamcode.opmodes.tuning

import com.acmerobotics.dashboard.FtcDashboard
import com.acmerobotics.dashboard.config.Config
import com.arcrobotics.ftclib.command.InstantCommand
import com.arcrobotics.ftclib.command.ParallelCommandGroup
import com.arcrobotics.ftclib.command.SequentialCommandGroup
import com.millburnx.utils.Vec2d
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import org.firstinspires.ftc.teamcode.common.Robot
import org.firstinspires.ftc.teamcode.common.subsystems.intake.Diffy
import org.firstinspires.ftc.teamcode.common.subsystems.intake.IntakeArmPosition
import org.firstinspires.ftc.teamcode.common.subsystems.vision.SampleVision
import org.firstinspires.ftc.teamcode.common.subsystems.vision.Vision
import org.firstinspires.ftc.teamcode.common.utils.EdgeDetector
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

open class SampleCameraRobot(
    opMode: OpMode,
) : Robot(opMode) {
    open val camera by lazy { SampleVision(this) }
    override val additionalSubsystems = listOf(camera)

    override fun init() {
        imu.resetYaw()
        super.init()
    }
}

@TeleOp(name = "Camera Tuner")
open class CameraTuner : OpMode() {
    override val robot by lazy { CameraRobot(this) }

    override fun exec() {
        robot.telemetry.addData("state 1", robot.camera.camera1.cameraState)
        robot.telemetry.addData("FPS 1", robot.camera.camera1.fps)

        robot.telemetry.addData("state 2", robot.camera.camera2.cameraState)
        robot.telemetry.addData("FPS 2", robot.camera.camera2.fps)
    }
}

@TeleOp(name = "Sample Dectector Tuner")
@Config
class SampleDectectionTuner : OpMode() {
    override val robot by lazy { SampleCameraRobot(this) }

    val triggers by lazy {
        object {
            val linkageExtend =
                EdgeDetector(
                    gamepad1::right_bumper,
                    this@SampleDectectionTuner,
                    ParallelCommandGroup(
                        robot.intake.extend(),
                        robot.intake.open(),
                        robot.outtake.base(),
                    ),
                )

            val linkageRetract =
                EdgeDetector(
                    gamepad1::left_bumper,
                    this@SampleDectectionTuner,
                    SequentialCommandGroup(
                        ParallelCommandGroup(
                            robot.intake.retract(),
                            robot.outtake.open(),
                            robot.outtake.base(),
                        ),
                    ),
                )

            val pickup =
                EdgeDetector(
                    gamepad1::dpad_down,
                    this@SampleDectectionTuner,
                    InstantCommand({
                        robot.intake.arm.state = IntakeArmPosition.FLOOR
                        robot.intake.diffy.pitch = Diffy.pickupPitch
                    }, robot.intake),
                )

            val clawToggle =
                EdgeDetector(
                    gamepad1::dpad_up,
                    this@SampleDectectionTuner,
                    InstantCommand({
                        robot.intake.close()
                    }),
                )

            val stream1 =
                EdgeDetector(
                    gamepad1::triangle,
                ) {
                    FtcDashboard.getInstance().startCameraStream(robot.camera.sampleDetector1, 0.0)
                }

            val stream2 =
                EdgeDetector(
                    gamepad1::cross,
                ) {
                    FtcDashboard.getInstance().startCameraStream(robot.camera.sampleDetector2, 0.0)
                }
        }
    }

    override fun initialize() {
        super.initialize()
        FtcDashboard.getInstance().startCameraStream(robot.camera.sampleDetector1, 0.0)
    }

    override fun exec() {
        triggers

        robot.telemetry.addData(
            "samples red 1",
            robot.camera.sampleDetector1.redSamples
                .get()
                .size,
        )
        robot.telemetry.addData(
            "samples yellow 1",
            robot.camera.sampleDetector1.yellowSamples
                .get()
                .size,
        )
        robot.telemetry.addData(
            "samples blue 1",
            robot.camera.sampleDetector1.blueSamples
                .get()
                .size,
        )

        robot.telemetry.addData(
            "samples red 2",
            robot.camera.sampleDetector2.redSamples
                .get()
                .size,
        )~
        robot.telemetry.addData(
            "samples yellow 2",
            robot.camera.sampleDetector2.yellowSamples
                .get()
                .size,
        )
        robot.telemetry.addData(
            "samples blue 2",
            robot.camera.sampleDetector2.blueSamples
                .get()
                .size,
        )

        robot.telemetry.addData("state 1", robot.camera.camera1.cameraState)
        robot.telemetry.addData("FPS 1", robot.camera.camera1.fps)

        robot.telemetry.addData("state 2", robot.camera.camera2.cameraState)
        robot.telemetry.addData("FPS 2", robot.camera.camera2.fps)

        val allSamples1 = robot.camera.sampleDetector1.allSamples
        val centered = allSamples1.minByOrNull { it.pos.distanceTo(Vec2d()) }
        if (centered != null) {
            robot.drive.robotCentric(centered.pos.y * kP, centered.pos.x * kP, 0.0)
        } else {
            robot.drive.robotCentric(0.0, 0.0, 0.0)
        }
        robot.telemetry.addData("centered", centered.toString())
    }

    companion object {
        @JvmField
        var kP: Double = 0.0
    }
}
