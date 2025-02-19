package org.firstinspires.ftc.teamcode.opmodes.auton

import android.os.Environment
import com.acmerobotics.dashboard.config.Config
import com.arcrobotics.ftclib.command.Command
import com.arcrobotics.ftclib.command.SequentialCommandGroup
import com.arcrobotics.ftclib.command.WaitCommand
import com.millburnx.utils.TSV
import com.millburnx.utils.Vec2d
import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import org.firstinspires.ftc.teamcode.common.Robot
import org.firstinspires.ftc.teamcode.common.subsystems.drive.PIDCommand
import org.firstinspires.ftc.teamcode.common.subsystems.drive.PIDManager
import org.firstinspires.ftc.teamcode.common.utils.OpMode
import org.firstinspires.ftc.teamcode.common.utils.Pose2d
import java.io.File

class AutonRobot(
    opMode: OpMode,
) : Robot(opMode) {
    val pidManager = PIDManager(this)
    override val additionalSubsystems = listOf(pidManager)

    override fun init() {
        imu.resetYaw()
        super.init()
    }
}

@Autonomous(name = "Specimen Auton", preselectTeleOp = "Basic Teleop")
@Config
@SuppressWarnings("detekt:MagicNumber", "detekt:SpreadOperator")
class SpecimenAuton : OpMode() {
    override val robot by lazy { AutonRobot(this) }

    override fun initialize() {
        super.initialize()
        robot.drive.pose = Pose2d(startingX, startingY, startingHeading)
        val commands = mutableListOf<Command>()
        robot.outtake.arm.periodic()
        robot.outtake.claw.close()
        robot.outtake.claw.periodic()
        robot.intake.arm.periodic()
        robot.intake.diffy.periodic()
        robot.intake.claw.periodic()

        commands.add(
            SequentialCommandGroup(
                PIDCommand(robot, Pose2d(Vec2d(scoreX, scoreY), 0.0)),
                WaitCommand(1000),
                PIDCommand(robot, Pose2d(Vec2d(sample1X, sample1Y), 0.0)),
                WaitCommand(1000),
                PIDCommand(robot, Pose2d(Vec2d(sample2X, sample2Y), 0.0)),
                WaitCommand(1000),
                PIDCommand(robot, Pose2d(Vec2d(sample3X, sample3Y), 0.0)),
            ),
        )

        schedule(SequentialCommandGroup(*commands.toTypedArray()))
    }

    fun loadPoints(file: String): List<Pose2d> {
        val csv = TSV.bufferedRead(File("${Environment.getExternalStorageDirectory().path}/Paths/$file.tsv"))
        val points: MutableList<Pose2d> = mutableListOf()
        for (item in csv) {
            points.add(Pose2d(item[0].toDouble(), item[1].toDouble(), item[2].toDouble()))
        }
        return points
    }

    fun pointToPath(points: List<Pose2d>): List<PIDCommand> = points.map { PIDCommand(robot, it) }

    override fun exec() {
    }

    companion object {
        @JvmField
        var startingX = -60.0

        @JvmField
        var startingY = -17.0

        @JvmField
        var startingHeading = 0.0

        @JvmField
        var scoreX = -36.0

        @JvmField
        var scoreY = 0.0

        @JvmField
        var sample1X = -48.0

        @JvmField
        var sample1Y = -36.0

        @JvmField
        var sample2X = -48.0

        @JvmField
        var sample2Y = -42.0

        @JvmField
        var sample3X = -48.0

        @JvmField
        var sample3Y = -48.0
    }
}
