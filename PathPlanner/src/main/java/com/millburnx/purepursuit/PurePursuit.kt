package com.millburnx.purepursuit

import com.acmerobotics.dashboard.canvas.Canvas
import com.acmerobotics.dashboard.telemetry.TelemetryPacket
import com.millburnx.utils.Bezier
import com.millburnx.utils.BezierIntersection
import com.millburnx.utils.Circle
import com.millburnx.utils.Intersection
import com.millburnx.utils.Utils
import com.millburnx.utils.Vec2d
import kotlin.math.abs

class PurePursuit(
    val path: List<Vec2d>,
    val lookahead: ClosedFloatingPointRange<Double>,
    val threshold: Double = 1.0,
) {
    val beziers: List<Bezier> = Utils.pathToBeziers(path)
    val lastPoint: Vec2d = path.last()
    var lastSegment: Int = 0
    var lastIntersection: BezierIntersection = BezierIntersection(path[0], beziers[0], 0.0)
    var currentLookahead = lookahead.start
    var isFinished = false

    fun getIntersections(lookahead: Circle, segments: List<Bezier>): List<BezierIntersection> {
        return segments.flatMap { it.intersections(lookahead) }
    }

    fun getTarget(
        intersections: List<BezierIntersection>,
        pos: Vec2d,
        heading: Double
    ): BezierIntersection {
        val closest = intersections.minByOrNull {
            abs(getAngleDiff((pos to heading), it.point))
        }
        return closest ?: lastIntersection
    }

    fun updateLastIntersection(lastIntersection: BezierIntersection) {
        this.lastIntersection = lastIntersection
        this.lastSegment = beziers.indexOf(lastIntersection.line)
    }

    fun calc(pos: Vec2d, heading: Double, dt: Double): PurePursuitData {
        val distanceToLast = pos.distanceTo(lastPoint)
        if (distanceToLast < threshold) {
            isFinished = true

            @Suppress("KotlinConstantConditions")
            return PurePursuitData(
                lastIntersection.point,
                isEnding = true,
                isFinished,
                beziers,
                listOf(),
                listOf(lastIntersection),
                currentLookahead
            )
        }
        if (distanceToLast <= currentLookahead) {
            return PurePursuitData(
                lastPoint,
                isEnding = true,
                isFinished,
                beziers,
                listOf(Bezier.fromLine(pos, lastPoint)),
                listOf(lastIntersection),
                currentLookahead
            )
        }
        val lookaheadCircle = Circle(pos, currentLookahead)
        val remainingPath = beziers.subList(lastSegment, beziers.size)
        val intersections = getIntersections(lookaheadCircle, remainingPath)
        val targetIntersection = getTarget(intersections, pos, heading)
        updateLastIntersection(targetIntersection)

        val targetCurvature = abs(targetIntersection.line.getCurvature(targetIntersection.t))
        val newLookahead =
            Utils.lerp(lookahead.endInclusive, lookahead.start, (targetCurvature * 20).coerceIn(0.0, 1.0))
        println("Target Curvature: ${targetCurvature * 20} | $newLookahead | $dt") // large = small lookahead, small = large lookahead
        currentLookahead = Utils.lerp(currentLookahead, newLookahead, (dt * 5).coerceIn(0.0, 1.0))

        return PurePursuitData(
            targetIntersection.point,
            isEnding = false,
            isFinished,
            beziers,
            remainingPath,
            intersections,
            currentLookahead
        )
    }

    companion object {
        fun getAngleDiff(a: Pair<Vec2d, Double>, b: Vec2d): Double {
            val (aPoint, aAngle) = a
            val diff = Utils.normalizeAngle(aPoint.angleTo(b))
            return Utils.normalizeAngle(diff - aAngle)
        }

        fun render(data: PurePursuitData, packet: TelemetryPacket, pose: Vec2d, addTelemetry: Boolean = true) {
            val (target, isEnding, isDone, path, remainingPath, intersections, lookahead) = data
            if (addTelemetry) {
                packet.put("pure_pursuit/is_done", isDone)
                packet.put("pure_pursuit/is_ending", isEnding)
                packet.put("pure_pursuit/is_done", isDone)
                packet.put("pure_pursuit/path", path)
                packet.put("pure_pursuit/remaining_path", remainingPath)
                packet.put("pure_pursuit/intersections", intersections)
            }
            val canvas = packet.fieldOverlay()
            val colors = listOf(
                Utils.Colors.red,
                Utils.Colors.blue,
                Utils.Colors.green,
                Utils.Colors.yellow
            )
            renderPath(canvas, path, colors)
            val rrPose = pose.toRR()
            canvas.strokeCircle(rrPose.x, rrPose.y, lookahead)
            renderIntersections(canvas, intersections, target, colors)
        }

        fun renderPath(canvas: Canvas, path: List<Bezier>, colors: List<String>) {
            var color = 0
            for (bezier in path) {
                val currentColor = colors[color % colors.size]
                val colorString = currentColor
                canvas.setStroke(colorString)
                bezier.draw(canvas)
                color++
            }
        }

        fun renderIntersections(
            canvas: Canvas,
            intersections: List<Intersection<Bezier>>,
            target: Vec2d,
            colors: List<String>
        ) {
            var color = 0
            for (intersection in intersections) {
                val currentColor = if (intersection.point == target) {
                    colors[color % colors.size]
                } else {
                    "#ffffff"
                }
                val colorString = currentColor
                val rrPose = intersection.point.toRR()
                canvas.setFill(colorString)
                    .fillCircle(rrPose.x, rrPose.y, 1.0)
                color++
            }
        }
    }
}

data class PurePursuitData(
    val target: Vec2d,
    val isEnding: Boolean,
    val isFinished: Boolean,
    val path: List<Bezier>,
    val remainingPath: List<Bezier>,
    val intersections: List<Intersection<Bezier>>,
    val lookahead: Double
)