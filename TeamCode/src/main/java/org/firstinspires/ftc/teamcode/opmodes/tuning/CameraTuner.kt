package org.firstinspires.ftc.teamcode.opmodes.tuning

import com.acmerobotics.dashboard.FtcDashboard
import com.acmerobotics.dashboard.config.Config
import com.arcrobotics.ftclib.command.InstantCommand
import com.arcrobotics.ftclib.command.ParallelCommandGroup
import com.arcrobotics.ftclib.command.SequentialCommandGroup
import com.arcrobotics.ftclib.command.WaitCommand
import com.millburnx.utils.Vec2d
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import org.firstinspires.ftc.teamcode.common.Robot
import org.firstinspires.ftc.teamcode.common.subsystems.drive.AutoPickup
import org.firstinspires.ftc.teamcode.common.subsystems.intake.Diffy
import org.firstinspires.ftc.teamcode.common.subsystems.intake.IntakeArmPosition
import org.firstinspires.ftc.teamcode.common.subsystems.intake.IntakeClaw
import org.firstinspires.ftc.teamcode.common.subsystems.vision.SampleVision
import org.firstinspires.ftc.teamcode.common.subsystems.vision.Vision
import org.firstinspires.ftc.teamcode.common.utils.EdgeDetector
import org.firstinspires.ftc.teamcode.common.utils.OpMode
import org.firstinspires.ftc.teamcode.common.utils.Pose2d
import org.firstinspires.ftc.teamcode.opmodes.teleop.MainTeleopBlue.Companion.intakeDuration
import org.firstinspires.ftc.teamcode.opmodes.tuning.DiffyTuner.Companion.roll
import kotlin.math.cos
import kotlin.math.sin

open class CameraRobot(
    opMode: OpMode,
) : Robot(opMode) {
    open val camera by lazy { Vision(this) }
    override val additionalSubsystems = listOf(camera)
}

open class SampleCameraRobot(
    opMode: OpMode,
) : Robot(opMode) {
    open val camera by lazy { SampleVision(this) }
    val autoPickup = AutoPickup(this)
    override val additionalSubsystems = listOf(camera, autoPickup)
}

@TeleOp(name = "Camera Tuner", group = "Tuning")
open class CameraTuner : OpMode() {
    override val robot by lazy { CameraRobot(this) }

    override fun exec() {
        robot.telemetry.addData("state 1", robot.camera.camera1.cameraState)
        robot.telemetry.addData("FPS 1", robot.camera.camera1.fps)
    }
}

@TeleOp(name = "Sample Dectector Tuner", group = "Tuning")
@Config
class SampleDectectionTuner : OpMode() {
    override val robot by lazy { SampleCameraRobot(this) }

    val triggers by lazy {
        object {
            val linkageExtend =
                EdgeDetector(
                    gamepad1::right_bumper,
                    SequentialCommandGroup(
                        ParallelCommandGroup(
                            robot.intake.extend(),
                            robot.intake.open(),
                            robot.outtake.base(),
                        ),
                        WaitCommand(intakeDuration),
                        InstantCommand({ isReady = true }),
                    ),
                )

            val linkageRetract =
                EdgeDetector(
                    gamepad1::left_bumper,
                    SequentialCommandGroup(
                        InstantCommand({ isReady = false }),
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
                    ParallelCommandGroup(
                        robot.intake.arm.floor(),
                        robot.intake.diffy.pickup(),
                    ),
                )

            val clawToggle =
                EdgeDetector(
                    gamepad1::dpad_up,
                    robot.intake.close(),
                )

            val stream1 =
                EdgeDetector(
                    gamepad1::triangle,
                ) {
                    FtcDashboard.getInstance().startCameraStream(robot.camera.sampleDetector, 0.0)
                    robot.camera.update()
                }

//            val stream2 =
//                EdgeDetector(
//                    gamepad1::cross,
//                ) {
//                    FtcDashboard.getInstance().startCameraStream(robot.camera.sampleDetector2, 0.0)
//                }

            val reset =
                EdgeDetector(gamepad1::circle) {
                    targetPoseInternal = null
                    isReady = robot.intake.linkage.target == 1.0
                }
            val setRoll =
                EdgeDetector(gamepad1::cross) {
                    if (targetAngle > angleThres) return@EdgeDetector
                    robot.intake.diffy.roll = targetAngle
                }
        }
    }

    var targetAngle = 0.0
    var rotationalOffset: Vec2d = Vec2d()
    var isReady = false

    override fun initialize() {
        super.initialize()
        FtcDashboard.getInstance().startCameraStream(robot.camera.sampleDetector, 0.0)
    }

    override fun exec() {
        triggers

        robot.telemetry.addData(
            "samples red 1",
            robot.camera.sampleDetector.redSamples
                .get()
                .size,
        )
        robot.telemetry.addData(
            "samples yellow 1",
            robot.camera.sampleDetector.yellowSamples
                .get()
                .size,
        )
        robot.telemetry.addData(
            "samples blue 1",
            robot.camera.sampleDetector.blueSamples
                .get()
                .size,
        )

        robot.telemetry.addData("state 1", robot.camera.camera1.cameraState)
        robot.telemetry.addData("FPS 1", robot.camera.camera1.fps)

        robot.telemetry.addData("claw", robot.intake.claw.state == IntakeClaw.State.OPEN)

        val allSamples1 = robot.camera.sampleDetector.allSamples
        val centered = allSamples1.minByOrNull { it.pos.distanceTo(Vec2d()) }
        robot.telemetry.addData("centered", centered.toString())
        robot.telemetry.addData("targetPose", targetPose.toString())

        if (targetPoseInternal == null) {
            if (centered != null && isReady) {
                targetPoseInternal = Pose2d(centered.pos.rotate(-90.0), 0.0)
                targetPoseInternalOffset = robot.drive.pose

                if (fullAuto) {
                    if (targetAngle <= angleThres) {
                        robot.intake.diffy.roll = targetAngle
                    }
                }
            }
        }

        println("${robot.drive.pidManager.atTarget()} | $isReady")

        if (fullAuto &&
            robot.intake.linkage.target == 1.0 &&
            robot.drive.pidManager.atTarget() &&
            targetPoseInternal != null &&
            robot.intake.arm.state == IntakeArmPosition.EXTENDED &&
            isReady
        ) {
            schedule(
                SequentialCommandGroup(
                    WaitCommand(250L),
                    robot.intake.grab(),
                    InstantCommand({ robot.intake.diffy.roll = Diffy.hoverRoll }),
                    robot.intake.retract(),
                    InstantCommand({
                        targetPoseInternal = null
                    }),
                ),
            )
            isReady = false
        }

        robot.drive.pidManager.isOn = translationEnabled
        if (!robot.drive.pidManager.isOn) {
            robot.drive.robotCentric(
                gamepad1.left_stick_y.toDouble(),
                -gamepad1.left_stick_x.toDouble(),
                -gamepad1.right_stick_x.toDouble(),
            )
        }
        if (targetPose != null) {
            if (rotationEnabled) {
                val newAngle = -Math.toRadians(robot.intake.diffy.roll * 90)
                rotationalOffset = Vec2d(cos(newAngle), sin(newAngle) - 1.0).rotate(-90.0) * clawRadius
                robot.drive.pidManager.target = targetPose!! - rotationalOffset
            } else {
                robot.drive.pidManager.target = targetPose!!
            }
        }

        if (centered != null) {
            roll = -centered.angle / 90.0
            robot.telemetry.addData("roll", roll)
            targetAngle = roll
        }
    }

    val targetPose: Pose2d?
        get() = if (targetPoseInternal == null) null else targetPoseInternalOffset + targetPoseInternal!! * scale

    var targetPoseInternal: Pose2d? = null
    var targetPoseInternalOffset: Pose2d = Pose2d()

    companion object {
        @JvmField
        var translationEnabled: Boolean = true

        @JvmField
        var rotationEnabled: Boolean = true

        @JvmField
        var scale: Double = 0.019

        @JvmField
        var angleThres: Double = .9

        @JvmField
        var clawRadius: Double = .5

        @JvmField
        var fullAuto: Boolean = true

        @JvmField
        var crosshairX: Double = 10.0

        @JvmField
        var crosshairY: Double = -10.0
    }
}
