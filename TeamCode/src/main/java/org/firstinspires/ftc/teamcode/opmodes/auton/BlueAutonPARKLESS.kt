package org.firstinspires.ftc.teamcode.opmodes.auton

import android.os.Environment
import com.acmerobotics.dashboard.FtcDashboard
import com.arcrobotics.ftclib.command.Command
import com.arcrobotics.ftclib.command.CommandOpMode
import com.arcrobotics.ftclib.command.InstantCommand
import com.arcrobotics.ftclib.command.SequentialCommandGroup
import com.arcrobotics.ftclib.command.WaitCommand
import com.arcrobotics.ftclib.controller.PIDController
import com.millburnx.utils.Path
import com.millburnx.utils.Vec2d
import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import org.firstinspires.ftc.teamcode.common.commands.PIDCommand
import org.firstinspires.ftc.teamcode.common.commands.PurePursuitCommand
import org.firstinspires.ftc.teamcode.common.subsystems.Arm
import org.firstinspires.ftc.teamcode.common.subsystems.Drive
import org.firstinspires.ftc.teamcode.common.subsystems.Intake
import org.firstinspires.ftc.teamcode.common.subsystems.Lift
import org.firstinspires.ftc.teamcode.common.utils.Telemetry
import org.firstinspires.ftc.teamcode.opmodes.tuning.APIDController
import org.firstinspires.ftc.teamcode.opmodes.tuning.RelativeDrive
import org.firstinspires.ftc.teamcode.opmodes.tuning.SpecimenDown
import org.firstinspires.ftc.teamcode.opmodes.tuning.SpecimenDown2
import org.firstinspires.ftc.teamcode.opmodes.tuning.SpecimenUp
import java.io.File
import java.lang.Error

@Autonomous(name = "Slower Auton")
class BlueAutonSeq : CommandOpMode() {
    val drive: Drive by lazy {
        Drive(
            hardwareMap,
            tel,
            dash,
            Vec2d(BlueAutonConfig.startX, BlueAutonConfig.startY),
            BlueAutonConfig.startH,
            zeroBreak = true
        )
    }
    val arm: Arm by lazy {
        Arm(hardwareMap, telemetry, lift.lift::getCurrentPosition)
    }
    val lift: Lift by lazy {
        Lift(hardwareMap)
    }
    val intake: Intake by lazy {
        Intake(hardwareMap)
    }
    val tel: Telemetry = Telemetry()
    val dash: FtcDashboard = FtcDashboard.getInstance()
    val paths = mutableListOf<Path>()

    val rootDir = Environment.getExternalStorageDirectory()

    fun loadSegment(index: Int): Path {
        return pathMap.getOrPut(index) {
            val path = "${rootDir}/Paths/${AutonConfig.pathName}${index}.tsv"
            val segment: Path = try {
                val loaded = Vec2d.loadList(File(path))
                println(loaded)
                loaded
            } catch (e: Error) {
                e.printStackTrace()
                println("PATH (`$path`) NOT FOUND")
                Path(listOf())
            }
            return segment
        }
    }

    companion object {
        val pathMap: MutableMap<Int, Path> = mutableMapOf()
    }

    val pidX = PIDController(AutonConfig.pidT_kP, AutonConfig.pidT_kI, AutonConfig.pidT_kD)
    val pidY = PIDController(AutonConfig.pidT_kP, AutonConfig.pidT_kI, AutonConfig.pidT_kD)
    val pidH = APIDController(AutonConfig.pidH_kP, AutonConfig.pidH_kI, AutonConfig.pidH_kD)


    fun purePursuitSegment(segment: Path) = PurePursuitCommand(
        drive,
        segment.points,
        segment.endHeading,
        dash,
        pidX,
        pidY,
        pidH,
        AutonConfig.minRange..AutonConfig.maxRange
    ).withTimeout(AutonConfig.timeout)

    fun pidSegment(
        endPoint: Vec2d,
        endHeading: Double? = null,
        multiF: Double = 1.0,
        multiH: Double = 1.0,
    ): PIDCommand =
        PIDCommand(drive, endPoint, endHeading, pidX, pidY, pidH, multiF, multiH)


    override fun initialize() {
        lift.armAngle = arm::angle
        intake.close()
        val commands: MutableList<Command> = mutableListOf()

        commands.add(InstantCommand({ arm.on() }))
//        commands.add(InstantCommand({ intake.close() }))
        commands.add(WaitCommand(100))

        commands.add(purePursuitSegment(loadSegment(0)).withTimeout(2000))
        commands.add(WaitCommand(100))

        commands.add(SpecimenUp(arm, lift, intake))
        commands.add(RelativeDrive(drive, AutonConfig.barPower).withTimeout(1000))
        commands.add(WaitCommand(200))
        commands.add(SpecimenDown(arm, lift, intake))
        commands.add(WaitCommand(100))
        commands.add(pidSegment(Vec2d(-36.0, 0.0), 0.0).withTimeout(1000))
        commands.add(WaitCommand(100))
        commands.add(
            SequentialCommandGroup(
                SpecimenDown2(arm, lift, intake),
                WaitCommand(1000),
                purePursuitSegment(loadSegment(1))
            )
        )
        commands.add(WaitCommand(200))
        commands.add(purePursuitSegment(loadSegment(2)).withTimeout(2250))
        commands.add(WaitCommand(200))
        commands.add(pidSegment(Vec2d(-54, 50), 45.0, 0.75, 1.0).withTimeout(2500)) // place 1st
        commands.add(WaitCommand(200))
        commands.add(pidSegment(Vec2d(-48, 42), 90.0, 0.75, 0.75).withTimeout(750))
        commands.add(WaitCommand(200))
        commands.add(pidSegment(Vec2d(-12, 42), 90.0, 0.75, 0.75).withTimeout(1250))
        commands.add(WaitCommand(200))
        commands.add(pidSegment(Vec2d(-12, 47), 90.0, 0.5, 0.75).withTimeout(500))
        commands.add(WaitCommand(200))
        commands.add(pidSegment(Vec2d(-56, 51), 90.5, 0.75, 1.0).withTimeout(2500)) // place 2d
        commands.add(WaitCommand(200))
        commands.add(pidSegment(Vec2d(-58, -46), 90.0, 0.75, 1.0).withTimeout(4500)) // park
        commands.add(WaitCommand(200))

        schedule(
            SequentialCommandGroup(*commands.toTypedArray())
        )
    }

    override fun run() {
        super.run()
    }
}