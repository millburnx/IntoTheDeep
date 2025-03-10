package org.firstinspires.ftc.teamcode.common.subsystems.drive

import com.acmerobotics.dashboard.config.Config
import com.arcrobotics.ftclib.command.CommandBase
import com.arcrobotics.ftclib.command.ConditionalCommand
import com.arcrobotics.ftclib.command.InstantCommand
import com.arcrobotics.ftclib.command.RunCommand
import com.arcrobotics.ftclib.command.SequentialCommandGroup
import com.arcrobotics.ftclib.command.WaitCommand
import com.arcrobotics.ftclib.command.WaitUntilCommand
import com.millburnx.utils.Vec2d
import com.qualcomm.robotcore.util.ElapsedTime
import org.firstinspires.ftc.teamcode.common.processors.SampleColor
import org.firstinspires.ftc.teamcode.common.subsystems.intake.Diffy
import org.firstinspires.ftc.teamcode.common.utils.Pose2d
import org.firstinspires.ftc.teamcode.common.utils.Subsystem
import org.firstinspires.ftc.teamcode.opmodes.tuning.SampleCameraRobot
import org.firstinspires.ftc.teamcode.opmodes.tuning.SampleDectectionTuner.Companion.clawRadius
import org.firstinspires.ftc.teamcode.opmodes.tuning.SampleDectectionTuner.Companion.scale

@Config
class AutoPickup(
    val robot: SampleCameraRobot,
) : Subsystem() {
    val colors
        get() =
            listOf(if (robot.isRed) SampleColor.RED else SampleColor.BLUE) +
                (if (robot.doYellow) listOf(SampleColor.YELLOW) else emptyList())

    var lastTarget: Pair<Pose2d, Double>? = null
        set(value) {
            field = value
            if (value == null) {
                lastTargetTimer = null
            } else {
                lastTargetTimer = ElapsedTime()
            }
        }
    var lastTargetTimer: ElapsedTime? = null

    var scanning: Boolean = false

    override fun periodic() {
        if (scanning) {
            val samples = robot.camera.sampleDetector.allSamples
            val filteredSamples = samples.filter { colors.contains(it.color) }
            if (filteredSamples.isNotEmpty()) {
                val target = filteredSamples.minBy { it.pos.distanceTo(Vec2d()) }

                println(target)

                val sampleOffset = target.pos * scale
                val currentPose = robot.drive.pose
                val actualAngle = target.angle + (robot.intake.diffy.roll - Diffy.hoverRoll) * 360
                val rotationalOffset =
                    ((Vec2d.fromAngle(Math.toRadians(actualAngle) + 90) * Vec2d(1, 1) + Vec2d(0, -1)) * clawRadius)

                val totalOffset = (sampleOffset - rotationalOffset).rotate(Math.toRadians(currentPose.degrees - 90.0))
                val targetPose = currentPose + totalOffset

                lastTarget = targetPose to (actualAngle / 360)

//                robot.intake.diffy.roll = Diffy.hoverRoll + actualAngle / 360
//                println("${Diffy.hoverRoll + actualAngle / 360} $actualAngle ${target.angle}")
//                if (target.angle > 1) {
//                    robot.intake.diffy.roll += rollSpeed * robot.deltaTime.deltaTime * target.angle
//                } else if (target.angle < -1) {
//                    robot.intake.diffy.roll += rollSpeed * robot.deltaTime.deltaTime * target.angle
//                }

                robot.telemetry.addData("target roll", Diffy.hoverRoll + actualAngle / 360)
                robot.telemetry.addData("actual angle", actualAngle)
                robot.telemetry.addData("target angle", target.angle)
            }
        }
        if (lastTargetTimer != null) {
            val duration = lastTargetTimer?.milliseconds() ?: Double.MAX_VALUE
            if (duration > cacheDuration) {
                lastTarget = null
            }
        }
    }

    fun startScanning() =
        InstantCommand({
            scanning = true
//            robot.intake.diffy.isManual = true
        })

    fun stopScanning() =
        InstantCommand({
            scanning = false
//            robot.intake.diffy.isManual = false
        })

    fun rumble() {
        if (lastTarget == null) return
        robot.opMode.gamepad1.rumble(
            (cacheDuration - 200)
                .coerceAtLeast(robot.deltaTime.deltaTime * 1000)
                .toInt(),
        )
    }

    fun rumbleForever() = RunCommand(this::rumble)

    fun cancelRumble(cmd: RunCommand) = InstantCommand({ if (cmd.isScheduled) cmd.cancel() })

    fun align(): CommandBase =
        ConditionalCommand(
            SequentialCommandGroup(
                InstantCommand({
                    val target = lastTarget!!
                    robot.drive.pidManager.isOn = true
                    robot.drive.pidManager.isSamplePickup = true
                    robot.drive.pidManager.target = target.first
                    robot.intake.diffy.pitch = Diffy.pickupPitch
                    robot.intake.diffy.roll = Diffy.hoverRoll + target.second
                }),
                WaitUntilCommand(robot.drive.pidManager::atTarget),
                WaitCommand(pidStablizationDuration),
            ),
            InstantCommand({}),
            { lastTarget != null },
        )

    fun stop() =
        InstantCommand({
            robot.drive.pidManager.isOn = false
            robot.drive.pidManager.isSamplePickup = false
        })

    companion object {
        @JvmField
        var cacheDuration: Double = 500.0

        @JvmField
        var cameraStablizationDuration: Long = 0

        @JvmField
        var pidStablizationDuration: Long = 250

        @JvmField
        var alignmentTimeout: Long = 1000

        @JvmField
        var rollSpeed: Double = 0.01
    }
}
