package org.firstinspires.ftc.teamcode.opmodes.auton

import android.os.Environment
import com.acmerobotics.dashboard.FtcDashboard
import com.acmerobotics.dashboard.config.Config
import com.arcrobotics.ftclib.command.Command
import com.arcrobotics.ftclib.command.CommandOpMode
import com.arcrobotics.ftclib.command.InstantCommand
import com.arcrobotics.ftclib.command.ParallelCommandGroup
import com.arcrobotics.ftclib.command.SequentialCommandGroup
import com.arcrobotics.ftclib.command.WaitCommand
import com.arcrobotics.ftclib.controller.PIDController
import com.millburnx.utils.Path
import com.millburnx.utils.Vec2d
import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import com.qualcomm.robotcore.hardware.Servo
import org.firstinspires.ftc.teamcode.common.commands.PIDCommand
import org.firstinspires.ftc.teamcode.common.commands.PickupGroup
import org.firstinspires.ftc.teamcode.common.commands.PurePursuitCommand
import org.firstinspires.ftc.teamcode.common.commands.RelativeDrive
import org.firstinspires.ftc.teamcode.common.commands.ReturnToBase
import org.firstinspires.ftc.teamcode.common.commands.SpecimenScore
import org.firstinspires.ftc.teamcode.common.subsystems.Arm
import org.firstinspires.ftc.teamcode.common.subsystems.Drive
import org.firstinspires.ftc.teamcode.common.subsystems.Intake
import org.firstinspires.ftc.teamcode.common.subsystems.Lift
import org.firstinspires.ftc.teamcode.common.subsystems.vision.SamplePipeline
import org.firstinspires.ftc.teamcode.common.subsystems.vision.VisionPortal
import org.firstinspires.ftc.teamcode.common.utils.APIDController
import org.firstinspires.ftc.teamcode.common.utils.Telemetry
import java.io.File
import java.lang.Error

@Config
object BlueAutonConfig {
    @JvmField
    var startX = -60.0

    @JvmField
    var startY = 12.0

    @JvmField
    var startH = 0.0

    @JvmField
    var timeout: Long = 1000

    @JvmField
    var specimenPower: Double = 0.25

    @JvmField
    var specimenDuration: Long = 750

    @JvmField
    var samplePower: Double = 0.25

    @JvmField
    var sampleDuration: Long = 500

    @JvmField
    var parkPower: Double = -0.375

    @JvmField
    var parkDuration: Long = 1000
}

@Autonomous(name = "Full Auton")
class BlueAuton : CommandOpMode() {
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

    val parkServo: Servo by lazy {
        hardwareMap["parkServo"] as Servo
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


    val samplePipeline: SamplePipeline by lazy { SamplePipeline() }
    val visionPortal: VisionPortal by lazy {
        VisionPortal(
            hardwareMap,
            "camera1",
            listOf(samplePipeline)
        )
    }

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
        threshold: Double = AutonConfig.threshold,
        thresholdHeading: Double = AutonConfig.headingTolerance,
    ): PIDCommand =
        PIDCommand(drive, endPoint, endHeading, pidX, pidY, pidH, multiF, multiH, threshold, thresholdHeading)


    override fun initialize() {
        //start at (-60, 12)
        lift.armAngle = arm::angle
        intake.close()
        val commands: MutableList<Command> = mutableListOf()

        commands.add(ReturnToBase(arm, lift))
        commands.add(pidSegment(Vec2d(-40, 0), 0.0, threshold = AutonConfig.threshold * 2))
        commands.add(WaitCommand(250))
        commands.add(SpecimenScore(arm, lift, intake))
        commands.add(WaitCommand(250))
        commands.add(RelativeDrive(drive, BlueAutonConfig.specimenPower).withTimeout(BlueAutonConfig.specimenDuration))
        commands.add(InstantCommand(intake::open, intake))
        commands.add(WaitCommand(250)) // spec 1 done
        commands.add(
            ParallelCommandGroup(
                ReturnToBase(arm, lift),
                pidSegment(Vec2d(-50, -31), -90.0, threshold = AutonConfig.threshold * 2)
            )
        )
        commands.add(WaitCommand(250))
        commands.add(pidSegment(Vec2d(-53, -33), -135.0, threshold = AutonConfig.threshold * 2))

        // UNTESTED
        // TODO: Test during comp (cope)
        commands.add(
            PickupGroup(drive, arm, lift, intake, visionPortal.cameraSize, samplePipeline.detections::get)
        )

        commands.add(pidSegment(Vec2d(-40, 0), 0.0, threshold = AutonConfig.threshold * 2))
        commands.add(WaitCommand(250))
        commands.add(SpecimenScore(arm, lift, intake))
        commands.add(WaitCommand(250))
        commands.add(RelativeDrive(drive, BlueAutonConfig.specimenPower).withTimeout(BlueAutonConfig.specimenDuration))
        commands.add(InstantCommand(intake::open, intake))
        commands.add(WaitCommand(250)) // spec 2 done
        commands.add(pidSegment(Vec2d(-50, 0), 0.0, threshold = AutonConfig.threshold * 2))
        commands.add(pidSegment(Vec2d(-50, 30), 0.0, threshold = AutonConfig.threshold * 2))
        commands.add(pidSegment(Vec2d(-50, 30), 0.0, threshold = AutonConfig.threshold * 2))
        commands.add(pidSegment(Vec2d(-12, 40), 90.0).withTimeout(2000)) // park??~???!?!?!?
        commands.add(WaitCommand(BlueAutonConfig.timeout))
        commands.add(RelativeDrive(drive, BlueAutonConfig.parkPower).withTimeout(BlueAutonConfig.parkDuration))
        commands.add(WaitCommand(BlueAutonConfig.timeout))
        commands.add(InstantCommand({ parkServo.position = 0.6 }))


//        commands.add(
//            ParallelCommandGroup(
//                ReturnToBase(arm, lift),
//                SequentialCommandGroup(
//                    pidSegment(Vec2d(-40, 0), 0.0),
//                    pidSegment(Vec2d(-36, 48), 0.0, 0.75, 0.75, threshold = AutonConfig.threshold * 4).withTimeout(3000)
//                )
//            )
//        )
//        commands.add(
//            PickupGroup(drive, arm, lift, intake, visionPortal.cameraSize, samplePipeline.detections::get)
//        )
//        commands.add(
//            pidSegment(Vec2d(-46, 51), 135.0, 0.75, 0.75, threshold = AutonConfig.threshold * 3).withTimeout(2000)
//        )
//        commands.add(
//            SampleScore(arm, lift, intake)
//        )
//        commands.add(WaitCommand(250))
//        commands.add(RelativeDrive(drive, BlueAutonConfig.samplePower).withTimeout(BlueAutonConfig.sampleDuration))
//        commands.add(WaitCommand(250))
//        commands.add(InstantCommand(intake::open))
//        commands.add(WaitCommand(BlueAutonConfig.timeout))
//        commands.add(pidSegment(Vec2d(-44, 47), 135.0).withTimeout(1000))
//        commands.add(ReturnToBase(arm, lift))
//        commands.add(pidSegment(Vec2d(-32, 60), 0.0).withTimeout(1000))
//        commands.add(WaitCommand(BlueAutonConfig.timeout))
//        commands.add(
//            PickupGroup(drive, arm, lift, intake, visionPortal.cameraSize, samplePipeline.detections::get)
//        )
//        commands.add(WaitCommand(250))
//        commands.add(
//            pidSegment(Vec2d(-46, 51), 135.0, 0.75, 0.75, threshold = AutonConfig.threshold * 3).withTimeout(2000)
//        )
//        commands.add(WaitCommand(250))
//        commands.add(
//            SampleScore(arm, lift, intake)
//        )
//        commands.add(WaitCommand(250))
//        commands.add(RelativeDrive(drive, BlueAutonConfig.samplePower).withTimeout(BlueAutonConfig.sampleDuration))
//        commands.add(InstantCommand(intake::open))
//        commands.add(WaitCommand(250))
//        commands.add(pidSegment(Vec2d(-44, 47), 135.0).withTimeout(1000))
//        commands.add(WaitCommand(BlueAutonConfig.timeout))
//        commands.add(ReturnToBase(arm, lift))
//        commands.add(WaitCommand(BlueAutonConfig.timeout))
//        commands.add(pidSegment(Vec2d(-12, 40), 90.0).withTimeout(2000))
//        commands.add(WaitCommand(BlueAutonConfig.timeout))
//        commands.add(RelativeDrive(drive, BlueAutonConfig.parkPower).withTimeout(BlueAutonConfig.parkDuration))
//        commands.add(WaitCommand(BlueAutonConfig.timeout))
//        commands.add(InstantCommand({ parkServo.position = 0.6 }))


        schedule(
            SequentialCommandGroup(*commands.toTypedArray())
        )
    }

    override fun run() {
        super.run()
    }
}