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
import org.firstinspires.ftc.teamcode.common.subsystems.vision.IDetection
import org.firstinspires.ftc.teamcode.common.utils.APIDController
import org.firstinspires.ftc.teamcode.opmodes.tuning.CameraPickup.Companion.kd
import org.firstinspires.ftc.teamcode.opmodes.tuning.CameraPickup.Companion.kdRot
import org.firstinspires.ftc.teamcode.opmodes.tuning.CameraPickup.Companion.ki
import org.firstinspires.ftc.teamcode.opmodes.tuning.CameraPickup.Companion.kiRot
import org.firstinspires.ftc.teamcode.opmodes.tuning.CameraPickup.Companion.kp
import org.firstinspires.ftc.teamcode.opmodes.tuning.CameraPickup.Companion.kpRot
import org.firstinspires.ftc.teamcode.opmodes.tuning.CameraPickup.Companion.maxRotation
import org.firstinspires.ftc.teamcode.opmodes.tuning.CameraPickup.Companion.maxSpeed
import org.firstinspires.ftc.teamcode.opmodes.tuning.CameraPickup.Companion.strafeMulti
import kotlin.math.abs
import kotlin.math.sign
import kotlin.math.sqrt

@Config
open class PickupCommand(
    val drive: Drive,
    val cameraSize: Vec2d,
    val getSamples: () -> List<IDetection>,
    val isClip: Boolean = false,
    val offset: Vec2d = Vec2d(0.0, 0.0),
) :
    CommandBase() {
    val xPID: PIDController by lazy { PIDController(kp, ki, kd) }
    val yPID: PIDController by lazy { PIDController(kp, ki, kd) }
    val rPID: APIDController by lazy { APIDController(kpRot, kiRot, kdRot) }

    // power instead of error cuz steady state error, so we don't get stuck
    var targetSample: IDetection? = null

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

            drive.robotCentric(yPower, xPower, rPower)
        } else {
            drive.robotCentric(0.0, 0.0, 0.0)
        }
        this.targetSample = targetSample
    }

    override fun isFinished(): Boolean {
        val targetSample = this.targetSample
        if (targetSample == null) return false
        val tolerance = Vec2d(1 * strafeMulti, 1.0) * if (isClip) tolClip else tol

        val cameraCenter = (cameraSize / 2).flip()
        val targetCenter = cameraCenter * (offset + 1)
        val sampleCenter = targetSample.pos.flip()

        val diff = targetCenter - sampleCenter
        val diffH = targetSample.angle

        val doneX = abs(diff.x) < tolerance.x
        val doneY = abs(diff.y) < tolerance.y
        val doneH = abs(diffH) < if (isClip) tolHClip else tolH
        return doneX && doneY && doneH
    }


    override fun end(interrupted: Boolean) {
        drive.robotCentric(0.0, 0.0, 0.0)
    }

    companion object {
        @JvmField
        var tol = 10

        @JvmField
        var tolClip = 10

        @JvmField
        var tolH = 10

        @JvmField
        var tolHClip = 2

        @JvmField
        var visionArm: Int = 70

//        @JvmField
//        val pickupArm: Int = 0

        @JvmField
        var clipOffset = 0

        @JvmField
        var sampleOffset = 0

        @JvmField
        var clipPostOffset = 100

        @JvmField
        var samplePostOffset = 0

        @JvmField
        var closeDelay: Long = 500

        @JvmField
        var squid: Boolean = true

        @JvmField
        var clipPower: Double = -0.175

        @JvmField
        var clipDelay: Long = 125

        @JvmField
        var clipLower: Double = Arm.kG / 3

        @JvmField
        var lowerDuration: Long = 500

        @JvmField
        var armUpDuration: Long = 500

        @JvmField
        var clipOffsetX: Double = 0.0

        @JvmField
        var clipOffsetY: Double = 0.25

        @JvmField
        var sampleOffsetX: Double = 0.0

        @JvmField
        var sampleOffsetY: Double = -0.5

        @JvmField
        var postClose: Long = 200

        @JvmField
        var duration: Long = 4000
    }
}

fun PickupGroup(
    drive: Drive,
    arm: Arm,
    lift: Lift,
    intake: Intake,
    cameraSize: Vec2d,
    samples: () -> List<IDetection>,
    isClip: Boolean = false
): CommandGroupBase {
    return SequentialCommandGroup(
        ParallelCommandGroup(
            LiftCommand(
                lift,
                Lift.base + if (isClip) PickupCommand.clipOffset else PickupCommand.sampleOffset
            ),
            ArmCommand(arm, PickupCommand.visionArm).withTimeout(PickupCommand.armUpDuration),
            InstantCommand(intake::open, intake),
        ),
        PickupCommand(
            drive,
            cameraSize,
            samples,
            isClip,
            if (isClip) Vec2d(
                PickupCommand.clipOffsetX,
                PickupCommand.clipOffsetY
            ) else Vec2d(PickupCommand.sampleOffsetX, PickupCommand.sampleOffsetY)
        ).withTimeout(PickupCommand.duration),
        if (isClip) LiftCommand(lift, Lift.base + PickupCommand.clipPostOffset) else LiftCommand(
            lift,
            Lift.base + PickupCommand.samplePostOffset
        ),
        SequentialCommandGroup(InstantCommand({
            arm.isOverride = true; arm.setPower(PickupCommand.clipLower)
        }), WaitCommand(PickupCommand.lowerDuration)),
        InstantCommand({ arm.isOverride = false; arm.off() }),
        // bend clip
        if (isClip) SequentialCommandGroup(InstantCommand({
            arm.isOverride = true; arm.setPower(PickupCommand.clipPower)
        }), WaitCommand(PickupCommand.clipDelay)) else InstantCommand({}),
        WaitCommand(PickupCommand.closeDelay),
        InstantCommand(intake::close, intake),
        WaitCommand(PickupCommand.postClose),
        // re-enable arm
        if (isClip) InstantCommand({
            arm.isOverride = false; arm.setPower(0.0)
        }) else InstantCommand({}),
        ArmCommand(arm, PickupCommand.visionArm),
    )
}