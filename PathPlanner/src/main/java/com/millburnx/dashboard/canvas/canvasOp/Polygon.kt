package com.millburnx.dashboard.canvas.canvasOp

import com.millburnx.dashboard.canvas.FTCDashboardImpl
import com.millburnx.utils.Scale
import java.awt.Graphics2D

class Polygon(
    private val xPoints: DoubleArray, private val yPoints: DoubleArray, private val stroke: Boolean,
) : CanvasOpImpl(Type.POLYGON) {
    override fun draw(ctx: FTCDashboardImpl.CanvasCtx, scale: Scale) {
        val xPoints = xPoints.map { scale.ppiScale(it) }.toIntArray()
        val yPoints = yPoints.map { scale.ppiScale(it) }.toIntArray()
        setColor(ctx)
        if (stroke) {
            return drawStroke(ctx.g2d, Pair(xPoints, yPoints))
        }
        return drawFill(ctx.g2d, Pair(xPoints, yPoints))
    }

    fun drawStroke(g2d: Graphics2D, points: Pair<IntArray, IntArray>) {
        val (xPoints, yPoints) = points
        g2d.drawPolygon(xPoints, yPoints, xPoints.size)
    }

    fun drawFill(g2d: Graphics2D, points: Pair<IntArray, IntArray>) {
        val (xPoints, yPoints) = points
        g2d.fillPolygon(xPoints, yPoints, xPoints.size)
    }
}