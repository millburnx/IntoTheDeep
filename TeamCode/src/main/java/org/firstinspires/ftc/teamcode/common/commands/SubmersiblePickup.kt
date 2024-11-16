package org.firstinspires.ftc.teamcode.common.commands

import com.arcrobotics.ftclib.command.CommandBase
import com.arcrobotics.ftclib.command.CommandGroupBase
import com.arcrobotics.ftclib.command.InstantCommand
import com.arcrobotics.ftclib.command.SequentialCommandGroup
import com.arcrobotics.ftclib.command.WaitCommand
import com.arcrobotics.ftclib.controller.PIDController
import com.millburnx.utils.Vec2d
import org.firstinspires.ftc.teamcode.common.subsystems.Arm
import org.firstinspires.ftc.teamcode.common.subsystems.Drive
import org.firstinspires.ftc.teamcode.common.subsystems.Intake
import org.firstinspires.ftc.teamcode.common.subsystems.Lift
import org.firstinspires.ftc.teamcode.common.subsystems.vision.SampleDetection
import kotlin.math.abs

class SubmersiblePickup(
    val drive: Drive,
    val lift: Lift,
    val cameraSize: Vec2d,
    val getSamples: () -> List<SampleDetection>,
    val offset: Vec2d = Vec2d(0.0, 0.0)
) : CommandBase() {
    val xPID: PIDController by lazy {
        PIDController(kp, ki, kd)
    }
    val yPID: PIDController by lazy {
        PIDController(kp, ki, kd)
    }

    var targetSample: SampleDetection? = null

    init {
        addRequirements(drive)
    }

    override fun execute() {
        lift.isOverride = true
        xPID.setPID(kp, ki, kd)
        yPID.setPID(kpS, kiS, kdS)

        val samples =
            getSamples().filter {
                if (abs(it.angle) > angleThreshold) return@filter false
                if (targetSample == null) return@filter true
                return@filter it.color == targetSample!!.color
            }
        val targetSample = samples.maxByOrNull { sample -> sample.boundingBox.area }

        if (targetSample != null) {
            this.targetSample = targetSample

            val cameraCenter = (cameraSize / 2).flip()
            val targetCenter = cameraCenter * (offset + 1)
            val sampleCenter = targetSample.pos.flip()

            val diff = targetCenter - sampleCenter

            var xPower = xPID.calculate(0.0, diff.x).coerceIn(-maxStrafeSpeed, maxStrafeSpeed)
            var yPower = yPID.calculate(0.0, diff.y).coerceIn(-maxSlideSpeed, maxSlideSpeed)

            drive.robotCentric(forwardForce, xPower, 0.0)
            lift.setPower(yPower)
        } else {
            drive.robotCentric(0.0, 0.0, 0.0)
        }
    }

    override fun end(interrupted: Boolean) {
        drive.robotCentric(0.0, 0.0, 0.0)
        lift.target = lift.position.toDouble()
        lift.isOverride = false
    }

    companion object {
        @JvmField
        var kp: Double = 0.0

        @JvmField
        var ki: Double = 0.0

        @JvmField
        var kd: Double = 0.0

        @JvmField
        var kpS: Double = 0.0

        @JvmField
        var kiS: Double = 0.0

        @JvmField
        var kdS: Double = 0.0

        @JvmField
        var angleThreshold: Double = 45.0

        @JvmField
        var maxStrafeSpeed: Double = 0.1

        @JvmField
        var maxSlideSpeed: Double = 0.1

        @JvmField
        var forwardForce: Double = 0.0

        @JvmField
        var offsetX: Double = 0.0

        @JvmField
        var offsetY: Double = -0.4

        @JvmField
        var lowerPower: Double = Arm.kG / 3.0

        @JvmField
        var lowerDuration: Long = 250

        @JvmField
        var postOffset: Double = 0.0

        @JvmField
        var closeDelay: Long = 250

        @JvmField
        var postClose: Long = 200

        @JvmField
        var PreArm: Int = 45

        @JvmField
        var PreArmDuration: Long = 1000

        @JvmField
        var PostArm: Int = 45

        @JvmField
        var PostArmDuration: Long = 1000
    }
}

fun SubmersibleGroup(
    drive: Drive,
    arm: Arm,
    lift: Lift,
    intake: Intake,
    cameraSize: Vec2d,
    samples: () -> List<SampleDetection>,
    isClip: Boolean = false
): CommandGroupBase {
    return SequentialCommandGroup(
        ReturnToBase(arm, lift),
        ArmCommand(arm, SubmersiblePickup.PreArm).withTimeout(SubmersiblePickup.PreArmDuration),
        LiftCommand(lift, 400.0),
        SubmersiblePickup(
            drive,
            lift,
            cameraSize,
            samples,
            Vec2d(SubmersiblePickup.offsetX, SubmersiblePickup.offsetY)
        ),
        LiftCommand(lift, lift.position + SubmersiblePickup.postOffset),
        SequentialCommandGroup(InstantCommand({
            arm.isOverride = true; arm.setPower(SubmersiblePickup.lowerPower)
        }), WaitCommand(SubmersiblePickup.lowerDuration)),

        InstantCommand({ arm.isOverride = false; arm.off() }),
        WaitCommand(SubmersiblePickup.closeDelay),
        InstantCommand(intake::close, intake),
        WaitCommand(SubmersiblePickup.postClose),
        // re-enable arm
        if (isClip) InstantCommand({
            arm.isOverride = false; arm.setPower(0.0)
        }) else InstantCommand({}),
        ArmCommand(arm, SubmersiblePickup.PostArm).withTimeout(SubmersiblePickup.PostArmDuration),
        LiftCommand(lift, Lift.base)
    )
}