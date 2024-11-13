package org.firstinspires.ftc.teamcode.common.commands

import com.acmerobotics.dashboard.config.Config
import com.arcrobotics.ftclib.command.CommandBase
import com.arcrobotics.ftclib.command.CommandGroupBase
import com.arcrobotics.ftclib.command.InstantCommand
import com.arcrobotics.ftclib.command.ParallelCommandGroup
import com.arcrobotics.ftclib.command.SequentialCommandGroup
import com.arcrobotics.ftclib.command.WaitCommand
import com.arcrobotics.ftclib.controller.PIDController
import com.millburnx.utils.Vec2d
import org.firstinspires.ftc.teamcode.common.subsystems.Arm
import org.firstinspires.ftc.teamcode.common.subsystems.Drive
import org.firstinspires.ftc.teamcode.common.subsystems.Intake
import org.firstinspires.ftc.teamcode.common.subsystems.Lift
import org.firstinspires.ftc.teamcode.common.subsystems.vision.Detection
import org.firstinspires.ftc.teamcode.common.utils.APIDController
import org.firstinspires.ftc.teamcode.opmodes.tuning.CameraPickup.Companion.kd
import org.firstinspires.ftc.teamcode.opmodes.tuning.CameraPickup.Companion.kdRot
import org.firstinspires.ftc.teamcode.opmodes.tuning.CameraPickup.Companion.ki
import org.firstinspires.ftc.teamcode.opmodes.tuning.CameraPickup.Companion.kiRot
import org.firstinspires.ftc.teamcode.opmodes.tuning.CameraPickup.Companion.kp
import org.firstinspires.ftc.teamcode.opmodes.tuning.CameraPickup.Companion.kpRot
import org.firstinspires.ftc.teamcode.opmodes.tuning.CameraPickup.Companion.maxRotation
import org.firstinspires.ftc.teamcode.opmodes.tuning.CameraPickup.Companion.maxSpeed
import org.firstinspires.ftc.teamcode.opmodes.tuning.CameraPickup.Companion.offsetX
import org.firstinspires.ftc.teamcode.opmodes.tuning.CameraPickup.Companion.offsetY
import org.firstinspires.ftc.teamcode.opmodes.tuning.CameraPickup.Companion.strafeMulti
import kotlin.math.abs
import kotlin.math.sign
import kotlin.math.sqrt

@Config
open class PickupCommand(
    val drive: Drive,
    val cameraSize: Vec2d,
    val getSamples: () -> List<Detection>
) :
    CommandBase() {
    val xPID: PIDController by lazy { PIDController(kp, ki, kd) }
    val yPID: PIDController by lazy { PIDController(kp, ki, kd) }
    val rPID: APIDController by lazy { APIDController(kpRot, kiRot, kdRot) }

    // power instead of error cuz steady state error, so we don't get stuck
    var lastPower = Vec2d(0.0, 0.0)
    var lastPowerH = 0.0
    var targetSample: Detection? = null

    init {
        addRequirements(drive)
    }

    override fun execute() {
        xPID.setPID(kp, ki, kd)
        yPID.setPID(kp, ki, kd)
        rPID.setPID(kpRot, kiRot, kdRot)

        val samples = getSamples()
        val targetSample = samples.maxByOrNull { sample -> sample.boundingBox.area }

        if (targetSample != null) {
            val offset = Vec2d(offsetX, offsetY)
            val cameraCenter = (cameraSize / 2).flip()
            val targetCenter = cameraCenter * (offset + 1)
            val sampleCenter = targetSample.pos.flip()

            val diff = targetCenter - sampleCenter
            val diffH = targetSample.angle

            var xPower = xPID.calculate(0.0, diff.x)
            var yPower = yPID.calculate(0.0, diff.y)
            var rPower = rPID.calculate(0.0, diffH)

            if (squid) {
                xPower = sqrt(abs(xPower)) * sign(xPower)
                yPower = sqrt(abs(yPower)) * sign(yPower)
                rPower = sqrt(abs(rPower)) * sign(rPower)
            }

            xPower = xPower.coerceIn(-maxSpeed, maxSpeed) * strafeMulti
            yPower = yPower.coerceIn(-maxSpeed, maxSpeed)
            rPower = rPower.coerceIn(-maxRotation, maxRotation)

            lastPower = Vec2d(xPower, yPower)
            lastPowerH = rPower

            drive.robotCentric(yPower, xPower, rPower)
        } else {
            drive.robotCentric(0.0, 0.0, 0.0)
        }
        this.targetSample = targetSample
    }

    override fun isFinished(): Boolean {
        if (targetSample == null) return false
        val minPower = Vec2d(kS * strafeMulti, kS)
        val doneX = abs(lastPower.x) < minPower.x
        val doneY = abs(lastPower.y) < minPower.y
        val doneH = abs(lastPowerH) < kSRot
        return doneX && doneY && doneH
    }


    override fun end(interrupted: Boolean) {
        drive.robotCentric(0.0, 0.0, 0.0)
    }

    companion object {
        @JvmField
        var kS = 0.05

        @JvmField
        var kSRot = 0.05

        @JvmField
        var visionArm: Int = 70

//        @JvmField
//        val pickupArm: Int = 0

        @JvmField
        var clipOffset = 0

        @JvmField
        var closeDelay: Long = 500

        @JvmField
        var squid: Boolean = true

        @JvmField
        var clipPower: Double = -0.5
    }
}

fun PickupGroup(
    drive: Drive,
    arm: Arm,
    lift: Lift,
    intake: Intake,
    cameraSize: Vec2d,
    samples: () -> List<Detection>,
    isClip: Boolean = false
): CommandGroupBase {
    return SequentialCommandGroup(
        ParallelCommandGroup(
            LiftCommand(lift, Lift.base + if (isClip) PickupCommand.clipOffset else 0),
            ArmCommand(arm, PickupCommand.visionArm),
            InstantCommand(intake::open, intake),
        ),
        PickupCommand(drive, cameraSize, samples),
        if (isClip) LiftCommand(lift, Lift.base) else InstantCommand({}),
        InstantCommand(arm::off),
        WaitCommand(PickupCommand.closeDelay),
        // bend clip
        if (isClip) InstantCommand({
            arm.isOverride = true; arm.setPower(PickupCommand.clipPower)
        }) else InstantCommand({}),
        InstantCommand(intake::close, intake),
        WaitCommand(100),
        // re-enable arm
        if (isClip) InstantCommand({
            arm.isOverride = false; arm.setPower(0.0)
        }) else InstantCommand({}),
        ArmCommand(arm, PickupCommand.visionArm),
    )
}