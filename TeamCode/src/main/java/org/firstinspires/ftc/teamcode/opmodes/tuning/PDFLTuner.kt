package org.firstinspires.ftc.teamcode.opmodes.tuning

import com.acmerobotics.dashboard.config.Config
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import org.firstinspires.ftc.teamcode.common.Robot
import org.firstinspires.ftc.teamcode.common.subsystems.drive.Drive
import org.firstinspires.ftc.teamcode.common.subsystems.drive.PIDManager
import org.firstinspires.ftc.teamcode.common.utils.OpMode
import org.firstinspires.ftc.teamcode.common.utils.Pose2d
import org.firstinspires.ftc.teamcode.common.utils.Subsystem
import org.firstinspires.ftc.teamcode.common.utils.control.PDFL
import org.firstinspires.ftc.teamcode.opmodes.auton.AutonRobot

class PDFLManager(
    robot: Robot,
) : PIDManager(robot) {
    val pdflX = PDFL(PDFLTuner.kP, PDFLTuner.kD, PDFLTuner.kF, PDFLTuner.kL, PDFLTuner.tolerance)
    val pdflY = PDFL(PDFLTuner.kP, PDFLTuner.kD, PDFLTuner.kF, PDFLTuner.kL, PDFLTuner.tolerance)
    val pdflH =
        PDFL(
            PDFLTuner.kPHeading,
            PDFLTuner.kDHeading,
            PDFLTuner.kFHeading,
            PDFLTuner.kLHeading,
            PDFLTuner.headingTolerance,
        )

    override fun periodic() {
        if (!isOn) return
        pdflX.iSquid = PDFLTuner.isSquid
        pdflX.kP = PDFLTuner.kP
        pdflX.kD = PDFLTuner.kD
        pdflX.kF = PDFLTuner.kF
        pdflX.kL = PDFLTuner.kL
        pdflX.tolerance = PDFLTuner.tolerance

        pdflY.iSquid = PDFLTuner.isSquid
        pdflY.kP = PDFLTuner.kP
        pdflY.kD = PDFLTuner.kD
        pdflY.kF = PDFLTuner.kF
        pdflY.kL = PDFLTuner.kL
        pdflY.tolerance = PDFLTuner.tolerance

        pdflH.iSquid = PDFLTuner.iSquidHeading
        pdflH.kP = PDFLTuner.kPHeading
        pdflH.kD = PDFLTuner.kDHeading
        pdflH.kF = PDFLTuner.kFHeading
        pdflH.kL = PDFLTuner.kLHeading
        pdflH.tolerance = PDFLTuner.headingTolerance

        val x = pdflX.calculate(robot.drive.pose.x, target.x, robot.deltaTime.deltaTime)
        val y = pdflY.calculate(robot.drive.pose.y, target.y, robot.deltaTime.deltaTime)
        val h = pdflH.calculateHeading(robot.drive.pose.heading, target.heading, robot.deltaTime.deltaTime)

        robot.drive.fieldCentric(-x, y, h)
    }
}

class PDFLDrive(
    robot: Robot,
) : Drive(robot) {
    override val pidManager: PDFLManager = PDFLManager(robot)
    override val subsystems: List<Subsystem> = listOf(stuckDectector, pidManager)
}

class PDFLBot(
    opMode: OpMode,
) : AutonRobot(opMode) {
    override val drive = PDFLDrive(this)
}

@TeleOp(name = "PDFL Tuner", group = "Tuning")
@Config
class PDFLTuner : OpMode() {
    override val robot by lazy { PDFLBot(this) }

    override fun exec() {
        robot.drive.pidManager.isOn = true
        robot.drive.pidManager.target = Pose2d(x, y, h)
        robot.telemetry.addData("at target", robot.drive.pidManager.atTarget())
    }

    companion object {
        @JvmField
        var x: Double = 0.0

        @JvmField
        var y: Double = 0.0

        @JvmField
        var h: Double = 0.0

        @JvmField
        var kP: Double = 0.0

        @JvmField
        var kD: Double = 0.0

        @JvmField
        var kF: Double = 0.0

        @JvmField
        var kL: Double = 0.0

        @JvmField
        var tolerance: Double = 0.0

        @JvmField
        var kPHeading: Double = 0.0

        @JvmField
        var kDHeading: Double = 0.0

        @JvmField
        var kFHeading: Double = 0.0

        @JvmField
        var kLHeading: Double = 0.0

        @JvmField
        var headingTolerance: Double = 0.0

        @JvmField
        var isSquid: Boolean = false

        @JvmField
        var iSquidHeading: Boolean = false
    }
}
