package com.millburnx.pathplanner.ui.components

import com.millburnx.pathplanner.ui.Theme
import javax.swing.JPanel
import javax.swing.border.MatteBorder

class View : JPanel() {
    init {
        border = MatteBorder(1, 1, 1, 1, Theme.borderColor)
        background = Theme.bg1
    }
}