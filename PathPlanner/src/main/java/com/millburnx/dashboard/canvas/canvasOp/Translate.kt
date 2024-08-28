package com.millburnx.dashboard.canvas.canvasOp

import com.millburnx.dashboard.canvas.FTCDashboardImpl
import com.millburnx.utils.Scale

class Translate(val x: Double, val y: Double) : CanvasOpImpl(Type.TRANSLATE) {
    override fun draw(ctx: FTCDashboardImpl.CanvasCtx, scale: Scale) {
        val (translationX, translationY) = scale.ppiScale(x to y)
        ctx.g2d.translate(translationX, translationY)
    }
}