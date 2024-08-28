package com.millburnx.dashboard.canvas.canvasOp

import com.millburnx.dashboard.canvas.FTCDashboardImpl
import com.millburnx.utils.Scale
import java.awt.Color

class Stroke(private val color: String) : CanvasOpImpl(Type.STROKE) {
    override fun draw(ctx: FTCDashboardImpl.CanvasCtx, scale: Scale) {
        ctx.stroke = Color.decode(color)
    }
}