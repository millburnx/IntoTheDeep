package com.millburnx.dashboard.canvas.canvasOp

import com.millburnx.dashboard.canvas.FTCDashboardImpl
import com.millburnx.utils.Scale
import java.awt.geom.AffineTransform

abstract class CanvasOpImpl(type: Type) : CanvasOp(type) {
    companion object {
        fun setColor(ctx: FTCDashboardImpl.CanvasCtx, stroke: Boolean = false) {
            if (stroke) {
                ctx.g2d.color = ctx.stroke
            } else {
                ctx.g2d.color = ctx.fill
            }
        }

        fun getTransform(
            ctx: FTCDashboardImpl.CanvasCtx,
            scale: Scale,
            transform: CanvasTransform,
        ): AffineTransform {
            val baseTransform = getBaseTransform(ctx, transform.usePageFrame)
            val (pivotX, pivotY) = scale.ppiFractionalScale(transform.pivotX to transform.pivotY)
            baseTransform.rotate(transform.theta, pivotX, pivotY)
            return baseTransform
        }

        private fun getBaseTransform(
            ctx: FTCDashboardImpl.CanvasCtx,
            usePageFrame: Boolean,
        ): AffineTransform {
            if (usePageFrame) {
                return ctx.g2d.transform.clone() as AffineTransform
            }
            return AffineTransform()
        }
    }

    abstract fun draw(ctx: FTCDashboardImpl.CanvasCtx, scale: Scale)
}