package org.firstinspires.ftc.teamcode.common.commands

import com.ThermalEquilibrium.homeostasis.Controllers.Feedback.PIDEx
import com.ThermalEquilibrium.homeostasis.Parameters.PIDCoefficientsEx
import com.acmerobotics.dashboard.config.Config
import com.arcrobotics.ftclib.command.CommandBase
import com.arcrobotics.ftclib.command.CommandGroupBase
import com.arcrobotics.ftclib.command.InstantCommand
import com.arcrobotics.ftclib.command.ParallelCommandGroup
import com.arcrobotics.ftclib.command.SequentialCommandGroup
import com.arcrobotics.ftclib.command.WaitCommand
import com.millburnx.utils.Vec2d
import org.firstinspires.ftc.teamcode.common.subsystems.Arm
import org.firstinspires.ftc.teamcode.common.subsystems.Drive
import org.firstinspires.ftc.teamcode.common.subsystems.Intake
import org.firstinspires.ftc.teamcode.common.subsystems.Lift
import org.firstinspires.ftc.teamcode.common.subsystems.vision.SampleDetection
import org.firstinspires.ftc.teamcode.opmodes.tuning.CameraPickup.Companion.kStab
import org.firstinspires.ftc.teamcode.opmodes.tuning.CameraPickup.Companion.kSum
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

@Config
class PickupCommand(
    val drive: Drive,
    val cameraSize: Vec2d,
    val getSamples: () -> List<SampleDetection>
) :
    CommandBase() {
    val xPID: PIDEx by lazy { PIDEx(PIDCoefficientsEx(kp, ki, kd, kSum, kStab, 0.3)) }
    val yPID: PIDEx by lazy { PIDEx(PIDCoefficientsEx(kp, ki, kd, kSum, kStab, 0.3)) }
    val rPID: PIDEx by lazy { PIDEx(PIDCoefficientsEx(kpRot, kiRot, kdRot, 0.25 / kiRot, 0.1, 0.3)) }

    // power instead of error cuz steady state error, so we don't get stuck
    var lastPower = Vec2d(0.0, 0.0)
    var lastPowerH = 0.0
    var targetSample: SampleDetection? = null

    init {
        addRequirements(drive)
    }

    override fun execute() {
        val samples = getSamples()
        val targetSample = samples.maxByOrNull { sample -> sample.boundingBox.area }

        if (targetSample != null) {
            val offset = Vec2d(offsetX, offsetY)
            val cameraCenter = (cameraSize / 2).flip()
            val targetCenter = cameraCenter * (offset + 1)
            val sampleCenter = targetSample.pos.flip()

            val diff = targetCenter - sampleCenter
            val diffH = targetSample.angle

            val xPower = xPID.calculate(0.0, diff.x).coerceIn(-maxSpeed, maxSpeed) * strafeMulti
            val yPower = yPID.calculate(0.0, diff.y).coerceIn(-maxSpeed, maxSpeed)
            val rPower = rPID.calculate(0.0, diffH).coerceIn(-maxRotation, maxRotation)

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

    companion object {
        @JvmField
        val kS = 0.05

        @JvmField
        val kSRot = 0.05

        @JvmField
        val visionArm: Int = 70

//        @JvmField
//        val pickupArm: Int = 0
    }
}

fun PickupGroup(
    drive: Drive,
    arm: Arm,
    lift: Lift,
    intake: Intake,
    cameraSize: Vec2d,
    samples: () -> List<SampleDetection>
): CommandGroupBase {
    return SequentialCommandGroup(
        ParallelCommandGroup(
            LiftCommand(lift, Lift.base),
            ArmCommand(arm, PickupCommand.visionArm),
            InstantCommand(intake::open, intake),
        ),
        PickupCommand(drive, cameraSize, samples),
//        ArmCommand(arm, PickupCommand.pickupArm),
        InstantCommand(arm::off),
        WaitCommand(1000),
        InstantCommand(intake::close, intake),
        WaitCommand(1000),
        ArmCommand(arm, PickupCommand.visionArm),
    )
}