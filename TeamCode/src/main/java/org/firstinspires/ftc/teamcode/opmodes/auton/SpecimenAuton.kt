package org.firstinspires.ftc.teamcode.opmodes.auton

import android.os.Environment
import com.acmerobotics.dashboard.config.Config
import com.arcrobotics.ftclib.command.Command
import com.arcrobotics.ftclib.command.InstantCommand
import com.arcrobotics.ftclib.command.ParallelCommandGroup
import com.arcrobotics.ftclib.command.ParallelDeadlineGroup
import com.arcrobotics.ftclib.command.SequentialCommandGroup
import com.arcrobotics.ftclib.command.WaitCommand
import com.millburnx.utils.TSV
import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import org.firstinspires.ftc.teamcode.common.Robot
import org.firstinspires.ftc.teamcode.common.commands.drive.RelativeDrive
import org.firstinspires.ftc.teamcode.common.commands.outtake.SlidesCommand
import org.firstinspires.ftc.teamcode.common.subsystems.drive.PIDCommand
import org.firstinspires.ftc.teamcode.common.subsystems.drive.PIDManager
import org.firstinspires.ftc.teamcode.common.subsystems.intake.Diffy
import org.firstinspires.ftc.teamcode.common.subsystems.intake.IntakeArmPosition
import org.firstinspires.ftc.teamcode.common.subsystems.outtake.OuttakeArmPosition
import org.firstinspires.ftc.teamcode.common.subsystems.outtake.OuttakeWristPosition
import org.firstinspires.ftc.teamcode.common.subsystems.outtake.Slides
import org.firstinspires.ftc.teamcode.common.utils.OpMode
import org.firstinspires.ftc.teamcode.common.utils.Pose2d
import org.firstinspires.ftc.teamcode.opmodes.teleop.BasicTeleop.Companion.outtakePickupClawDelay
import org.firstinspires.ftc.teamcode.opmodes.teleop.BasicTeleop.Companion.specScoreDelay
import java.io.File

class AutonRobot(opMode: OpMode) : Robot(opMode) {
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
        robot.intake.arm.state = IntakeArmPosition.SPECIMEN
        robot.intake.arm.periodic()
        robot.intake.diffy.roll = Diffy.specimenRoll
        robot.intake.diffy.pitch = Diffy.specimenPitch
        robot.intake.diffy.periodic()
        robot.intake.claw.periodic()
        val push1 = loadPoints("push1")

        val scoreSpecimen = {
            SequentialCommandGroup(
                ParallelDeadlineGroup(
                    WaitCommand(specScoreDelay),
                    InstantCommand({
                        robot.outtake.arm.state = OuttakeArmPosition.SPECIMEN
                        robot.outtake.wrist.state = OuttakeWristPosition.SPECIMEN
                    }, robot.outtake),
                    RelativeDrive(robot.drive, robot.pidManager, Pose2d(-0.5, 0.0, 0.0)),
                ),
                SlidesCommand(robot.outtake.slides, Slides.highRung),
                robot.outtake.open(),
                ParallelCommandGroup(
                    robot.outtake.base()
                )
            )
        }

        val pickupSpecimen = {
            SequentialCommandGroup(
                PIDCommand(robot, Pose2d(pickupX, pickupY, 180.0)),
                ParallelCommandGroup(
                    WaitCommand(humanPlayer),
                    InstantCommand({
                        robot.outtake.arm.state = OuttakeArmPosition.OUT
                        robot.outtake.wrist.state = OuttakeWristPosition.OUT
                    }, robot.outtake),
                    robot.outtake.open()
                ),
                RelativeDrive(robot.drive, robot.pidManager, Pose2d(0.5, 0.0, 0.0)).withTimeout(pickupDuration),
                SequentialCommandGroup(
                    robot.outtake.close(),
                    WaitCommand(outtakePickupClawDelay),
                    SlidesCommand(robot.outtake.slides, Slides.wall),
                    ParallelCommandGroup(
                        InstantCommand({
                            robot.outtake.arm.state = OuttakeArmPosition.BASKET
                            robot.outtake.wrist.state = OuttakeWristPosition.BASKET
                        }, robot.outtake),
                        PIDCommand(robot, Pose2d(pickupX, pickupY, 180.0)),
                    )
                )
            )
        }

        commands.add(
            SequentialCommandGroup(
                PIDCommand(robot, Pose2d(-34.0, -2.0, 180.0)),
                scoreSpecimen(),
                *pointToPath(push1).toTypedArray(),
                pickupSpecimen(),
                PIDCommand(robot, Pose2d(-40.0, -3.0, 180.0)),
                PIDCommand(robot, Pose2d(-34.0, -3.0, 180.0)),
                scoreSpecimen(),
                pickupSpecimen(),
                PIDCommand(robot, Pose2d(-40.0, -1.5, 180.0)),
                PIDCommand(robot, Pose2d(-34.0, -1.5, 180.0)),
                scoreSpecimen(),
                ParallelCommandGroup(
                    PIDCommand(robot, Pose2d(parkX, parkY, parkH)),
                    SequentialCommandGroup(
                        WaitCommand(parkExtendDelay),
                        robot.intake.sweepExtend()
                    )
                )
            )
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

    fun pointToPath(points: List<Pose2d>): List<PIDCommand> {
        return points.map { PIDCommand(robot, it) }
    }

    override fun exec() {
    }

    companion object {
        @JvmField
        var startingX = -60.0

        @JvmField
        var startingY = -17.0

        @JvmField
        var parkX = -48.0

        @JvmField
        var parkY = -32.0

        @JvmField
        var parkH = -135.0

        @JvmField
        var startingHeading = 0.0

        @JvmField
        var humanPlayer: Long = 500

        @JvmField
        var pickupDuration: Long = 750

        @JvmField
        var parkExtendDelay: Long = 500

        @JvmField
        var pickupX: Double = -42.0

        @JvmField
        var pickupY: Double = -46.0
    }
}