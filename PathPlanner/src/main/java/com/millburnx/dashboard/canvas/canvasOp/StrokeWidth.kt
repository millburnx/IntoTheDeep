package com.millburnx.dashboard.canvas.canvasOp

import com.millburnx.dashboard.canvas.FTCDashboardImpl
import com.millburnx.utils.Scale
import java.awt.BasicStroke

class StrokeWidth(private val width: Int) : CanvasOpImpl(Type.STROKE_WIDTH) {
    override fun draw(ctx: FTCDashboardImpl.CanvasCtx, scale: Scale) {
        ctx.g2d.stroke = BasicStroke(scale.ppiScale(width).toFloat())
    }
}