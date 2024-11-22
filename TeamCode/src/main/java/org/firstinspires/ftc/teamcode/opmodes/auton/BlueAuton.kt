package org.firstinspires.ftc.teamcode.opmodes.auton

import android.os.Environment
import com.acmerobotics.dashboard.FtcDashboard
import com.acmerobotics.dashboard.config.Config
import com.arcrobotics.ftclib.command.Command
import com.arcrobotics.ftclib.command.CommandOpMode
import com.arcrobotics.ftclib.command.InstantCommand
import com.arcrobotics.ftclib.command.ParallelCommandGroup
import com.arcrobotics.ftclib.command.SequentialCommandGroup
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
import org.firstinspires.ftc.teamcode.common.subsystems.vision.FasterSampleDetection
import org.firstinspires.ftc.teamcode.common.subsystems.vision.VisionPortal
import org.firstinspires.ftc.teamcode.common.utils.APIDController
import org.firstinspires.ftc.teamcode.common.utils.Telemetry
import java.io.File

@Config
object BlueAutonConfig {
    @JvmField
    var startX = -60.0

    @JvmField
    var startY = -12.0

    @JvmField
    var startH = 0.0

    @JvmField
    var specimenPower: Double = 0.25

    @JvmField
    var specimenDuration: Long = 1000

    @JvmField
    var pickupX: Double = -52.0

    @JvmField
    var pickupY: Double = -40.0
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


    val samplePipeline: FasterSampleDetection by lazy { FasterSampleDetection(telemetry) }
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
        PIDCommand(
            drive,
            endPoint,
            endHeading,
            pidX,
            pidY,
            pidH,
            multiF,
            multiH,
            threshold,
            thresholdHeading
        )


    override fun initialize() {
        //start at (-60, 12)
        lift.armAngle = arm::angle
        intake.close()
        visionPortal
        FtcDashboard.getInstance().startCameraStream(samplePipeline, 0.0)

        val commands: MutableList<Command> = mutableListOf()

        val score = SequentialCommandGroup(
            pidSegment(Vec2d(-45, -10), 0.0),
            SpecimenScore(arm, lift, intake),
            RelativeDrive(
                drive,
                BlueAutonConfig.specimenPower
            ).withTimeout(BlueAutonConfig.specimenDuration),
            InstantCommand(intake::open),
            ParallelCommandGroup(
                pidSegment(Vec2d(-45, -10), 0.0),
                ReturnToBase(arm, lift)
            )
        )
        val pickup = SequentialCommandGroup(
            pidSegment(
                Vec2d(BlueAutonConfig.pickupX, BlueAutonConfig.pickupY),
                -90.0,
            ),
            PickupGroup(drive, arm, lift, intake, visionPortal.cameraSize, samplePipeline.detections::get, true),
            ReturnToBase(arm, lift)
        )

        commands.add(ReturnToBase(arm, lift))
        commands.add(score)
        commands.add(pickup)
        commands.add(score)
        commands.add(pickup)
        commands.add(score)
        schedule(
            SequentialCommandGroup(*commands.toTypedArray())
        )
    }

    override fun run() {
        super.run()
    }
}