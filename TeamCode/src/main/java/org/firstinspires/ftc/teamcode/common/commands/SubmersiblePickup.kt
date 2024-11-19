package org.firstinspires.ftc.teamcode.common.commands

import com.acmerobotics.dashboard.config.Config
import com.arcrobotics.ftclib.command.CommandBase
import com.arcrobotics.ftclib.command.CommandGroupBase
import com.arcrobotics.ftclib.command.InstantCommand
import com.arcrobotics.ftclib.command.SequentialCommandGroup
import com.arcrobotics.ftclib.command.WaitCommand
import com.arcrobotics.ftclib.controller.PIDController
import com.millburnx.utils.Vec2d
import org.firstinspires.ftc.robotcore.external.Telemetry
import org.firstinspires.ftc.teamcode.common.subsystems.Arm
import org.firstinspires.ftc.teamcode.common.subsystems.Drive
import org.firstinspires.ftc.teamcode.common.subsystems.Intake
import org.firstinspires.ftc.teamcode.common.subsystems.Lift
import org.firstinspires.ftc.teamcode.common.subsystems.vision.IDetection
import org.firstinspires.ftc.teamcode.common.subsystems.vision.SampleColor
import org.firstinspires.ftc.teamcode.common.subsystems.vision.SampleDetection
import kotlin.math.abs

@Config
open class SubmersiblePickup(
    val drive: Drive,
    val lift: Lift,
    val cameraSize: Vec2d,
    val getSamples: () -> List<IDetection>,
    val offset: Vec2d = Vec2d(0.0, 0.0),
    val telemetry: Telemetry? = null
) : CommandBase() {
    val xPID: PIDController by lazy {
        PIDController(kp, ki, kd)
    }
    val yPID: PIDController by lazy {
        PIDController(kp, ki, kd)
    }

    var targetSample: SampleDetection? = null
    var targetColor: SampleColor? = null
    var lastSample: SampleDetection? = null

    init {
        addRequirements(drive)
    }

    override fun execute() {
        xPID.setPID(kp, ki, kd)
        yPID.setPID(kpS, kiS, kdS)

        val samples = getSamples()

        val samplesFiltered =
            samples.filter {
                true
//                if (abs(it.angle) > angleThreshold) return@filter false
//                if (targetColor == null) return@filter true
//                return true
//                return@filter it.color == targetColor!!
            }
        val targetSample = samplesFiltered.maxByOrNull { sample -> sample.boundingBox.area }

        telemetry?.addData("samples", samples.size)
        telemetry?.addData("samplesFiltered", samplesFiltered.size)

//        this.targetSample = targetSample

        if (targetSample != null) {
            lift.isOverride = true

//            this.targetColor = targetSample.color

            val cameraCenter = (cameraSize / 2).flip()
            val targetCenter = cameraCenter * (offset + 1)
            val sampleCenter = targetSample.pos.flip()

            val diff = targetCenter - sampleCenter

            var xPower = xPID.calculate(0.0, diff.x).coerceIn(-maxStrafeSpeed, maxStrafeSpeed)
            var yPower = yPID.calculate(0.0, diff.y).coerceIn(-maxSlideSpeed, maxSlideSpeed)

            telemetry?.addData("target", targetSample)
            telemetry?.addData("xPower", xPower)
            telemetry?.addData("yPower", yPower)
            telemetry?.addData("diff", diff)

            drive.robotCentric(forwardForce, xPower, 0.0)

            lift.setPower(yPower)
        } else {
            drive.robotCentric(0.0, 0.0, 0.0)
            lift.setPower(0.0)
        }

//        lastSample = targetSample

        telemetry?.update()
    }

    override fun end(interrupted: Boolean) {
        drive.robotCentric(0.0, 0.0, 0.0)
        lift.target = lift.position.toDouble()
        lift.isOverride = false
        targetSample = null
        lastSample = null
    }

    override fun isFinished(): Boolean {
        val targetSample = this.targetSample
        println("TS $targetSample")
        if (targetSample == null) return false
        val tolerance = Vec2d(1.0, 1.0) * tol

        val cameraCenter = (cameraSize / 2).flip()
        val targetCenter = cameraCenter * (offset + 1)
        val sampleCenter = targetSample.pos.flip()

        val diff = targetCenter - sampleCenter
        val diffH = targetSample.angle

        val doneX = abs(diff.x) < tolerance.x
        val doneY = abs(diff.y) < tolerance.y
        val doneH = abs(diffH) < tolH
        println("$doneX $doneY $doneH $diff")
        return doneX && doneY && doneH
    }

    companion object {
        @JvmField
        var kp: Double = -0.005

        @JvmField
        var ki: Double = 0.0

        @JvmField
        var kd: Double = 0.0

        @JvmField
        var kpS: Double = -0.005

        @JvmField
        var kiS: Double = 0.0

        @JvmField
        var kdS: Double = 0.0

        @JvmField
        var angleThreshold: Double = 45.0

        @JvmField
        var maxStrafeSpeed: Double = 0.25

        @JvmField
        var maxSlideSpeed: Double = 0.2

        @JvmField
        var forwardForce: Double = 0.1

        @JvmField
        var offsetX: Double = 0.0

        @JvmField
        var offsetY: Double = 0.25

        @JvmField
        var lowerPower: Double = Arm.kG / 3.0

        @JvmField
        var lowerDuration: Long = 250

        @JvmField
        var postOffset: Double = -100.0

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

        @JvmField
        var startingLift: Double = 400.0

        @JvmField
        var tol: Double = 40.0

        @JvmField
        var tolH: Double = 100000.0
    }
}

fun SummersibleEnter(
    drive: Drive,
    arm: Arm,
    lift: Lift,
    intake: Intake,
    cameraSize: Vec2d,
    samples: () -> List<IDetection>,
    tel: Telemetry
): CommandGroupBase {
    return SequentialCommandGroup(
        ReturnToBase(arm, lift),
        ArmCommand(arm, SubmersiblePickup.PreArm).withTimeout(SubmersiblePickup.PreArmDuration),
        LiftCommand(lift, SubmersiblePickup.startingLift),
    )
}

fun SubmersibleGroup(
    drive: Drive,
    arm: Arm,
    lift: Lift,
    intake: Intake,
    cameraSize: Vec2d,
    samples: () -> List<IDetection>,
    tel: Telemetry
): CommandGroupBase {
    return SequentialCommandGroup(
        SubmersiblePickup(
            drive,
            lift,
            cameraSize,
            samples,
            Vec2d(SubmersiblePickup.offsetX, SubmersiblePickup.offsetY),
            tel
        ),
        LiftCommand(lift, { lift.position + SubmersiblePickup.postOffset }),
        SequentialCommandGroup(InstantCommand({
            arm.isOverride = true; arm.setPower(SubmersiblePickup.lowerPower)
        }), WaitCommand(SubmersiblePickup.lowerDuration)),
        InstantCommand({ arm.isOverride = false; arm.off() }),
        WaitCommand(SubmersiblePickup.closeDelay),
        InstantCommand(intake::close, intake),
        WaitCommand(SubmersiblePickup.postClose),
        ArmCommand(arm, SubmersiblePickup.PostArm).withTimeout(SubmersiblePickup.PostArmDuration),
        LiftCommand(lift, Lift.base)
    )
}