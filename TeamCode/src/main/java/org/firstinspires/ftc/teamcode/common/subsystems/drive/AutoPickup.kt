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
import org.firstinspires.ftc.teamcode.common.utils.Pose2d
import org.firstinspires.ftc.teamcode.common.utils.Subsystem
import org.firstinspires.ftc.teamcode.opmodes.tuning.SampleCameraRobot
import org.firstinspires.ftc.teamcode.opmodes.tuning.SampleDectectionTuner.Companion.angleThres
import org.firstinspires.ftc.teamcode.opmodes.tuning.SampleDectectionTuner.Companion.clawRadius
import org.firstinspires.ftc.teamcode.opmodes.tuning.SampleDectectionTuner.Companion.scale

@Config
class AutoPickup(
    val robot: SampleCameraRobot,
    val colors: List<SampleColor>,
) : Subsystem() {
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
                val targetRoll = -target.angle / 90.0
                val actualRoll = if (targetRoll > angleThres) -1.0 else targetRoll
                val actualRadian = Math.toRadians(-actualRoll * 90.0) // reverse the angle to roll
                val rotationalOffset = ((Vec2d.fromAngle(actualRadian) + Vec2d(0, -1)) * clawRadius)

                val totalOffset = (sampleOffset - rotationalOffset).rotate(Math.toRadians(currentPose.degrees - 90.0))
                val targetPose = currentPose + totalOffset

                lastTarget = targetPose to actualRoll
            }
        }
        if (lastTargetTimer != null) {
            val duration = lastTargetTimer?.milliseconds() ?: Double.MAX_VALUE
            if (duration > cacheDuration) {
                lastTarget = null
            }
        }
    }

    fun startScanning() = InstantCommand({ scanning = true })

    fun stopScanning() = InstantCommand({ scanning = false })

    fun rumble() {
        if (lastTarget == null) return
        robot.opMode.gamepad1.rumble(
            (cacheDuration - 200)
                .coerceAtLeast(robot.deltaTime.deltaTime * 1000)
                .toInt(),
        )
    }

    val rumbleForever = RunCommand(this::rumble)

    fun cancelRumble() = InstantCommand({ if (rumbleForever.isScheduled) rumbleForever.cancel() })

    fun align(): CommandBase =
        ConditionalCommand(
            SequentialCommandGroup(
                InstantCommand({
                    val target = lastTarget!!
                    robot.pidManager.isOn = true
                    robot.pidManager.target = target.first
                    robot.intake.diffy.roll = target.second
                }),
                WaitUntilCommand(robot.pidManager::atTarget),
                WaitCommand(pidStablizationDuration),
            ),
            InstantCommand({}),
            { lastTarget != null },
        )

    fun stop() =
        InstantCommand({
            robot.pidManager.isOn = false
        })

    companion object {
        @JvmField
        var cacheDuration: Double = 500.0

        @JvmField
        var cameraStablizationDuration: Long = 250

        @JvmField
        var pidStablizationDuration: Long = 250

        @JvmField
        var alignmentTimeout: Long = 1000
    }
}
