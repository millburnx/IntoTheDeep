package com.millburnx.pathPlanner.components.editor

import com.millburnx.pathPlanner.components.editor.row.Row
import com.millburnx.pathPlanner.components.editor.row.RowModel
import com.millburnx.pathPlanner.components.editor.row.RowRenderer
import javax.swing.DropMode
import javax.swing.JList
import javax.swing.ListSelectionModel

public class EditorList(public val model: RowModel = RowModel()) : JList<Row>(model) {
    init {
        dropMode = DropMode.INSERT
        dragEnabled = true
        selectionMode = ListSelectionModel.SINGLE_SELECTION
        cellRenderer = RowRenderer()
        transferHandler = EditorTransferHandle(this)
    }
}