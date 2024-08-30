package com.millburnx.pathplanner.ui.components.editor.row

import com.millburnx.pathplanner.ui.Theme
import com.millburnx.utils.vec2d.Vec2dInt
import java.awt.GridBagConstraints
import java.awt.GridBagLayout
import java.awt.datatransfer.DataFlavor
import java.io.Serializable
import javax.swing.JComponent
import javax.swing.JPanel

class Row(val children: List<JComponent>) : JPanel(), Serializable {
    var index = -1
        set(value) {
            background = if (value % 2 == 0) Theme.bg1 else Theme.bg0
        }

    companion object {
        val FLAVOR = DataFlavor(JPanel::class.java, "JPanel")
    }

    constructor(vararg children: JComponent) : this(children.toList())

    val padding = Vec2dInt(12, 8)

    init {
        layout = GridBagLayout()
        children.forEachIndexed { index, component ->
            add(component, getConstraints(index))
        }
    }

    fun updateCellMargins(maxWidths: List<Int>) {
        children.forEachIndexed { index, component ->
            val diff = maxWidths[index] - component.preferredSize.width
            val gridBag = layout as GridBagLayout
            val constraints = getConstraints(index, diff.coerceAtLeast(0))
            gridBag.setConstraints(component, constraints)
        }
    }

    fun getConstraints(index: Int, offset: Int = 0): GridBagConstraints {
        val constraints = GridBagConstraints()
        constraints.gridy = 0
        constraints.gridx = index
        constraints.anchor = GridBagConstraints.BASELINE_LEADING
        constraints.weightx = 1.0
        constraints.insets.set(padding.y, padding.x, padding.y, padding.x + offset)
        return constraints
    }
}
