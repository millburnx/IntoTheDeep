package com.millburnx.dashboard.canvas.canvasOp

import com.millburnx.dashboard.canvas.FTCDashboardImpl
import com.millburnx.utils.Scale
import java.awt.Graphics2D

class Polyline(private val xPoints: DoubleArray, private val yPoints: DoubleArray) : CanvasOpImpl(Type.POLYLINE) {
    override fun draw(ctx: FTCDashboardImpl.CanvasCtx, scale: Scale) {
        val xPoints = xPoints.map { scale.ppiScale(it) }.toIntArray()
        val yPoints = yPoints.map { scale.ppiScale(it) }.toIntArray()
        setColor(ctx)
        drawStroke(ctx.g2d, scale, Pair(xPoints, yPoints))
    }

    fun drawStroke(g2d: Graphics2D, scale: Scale, points: Pair<IntArray, IntArray>) {
        val (xPoints, yPoints) = points
        g2d.drawPolyline(xPoints, yPoints, xPoints.size)
    }
}