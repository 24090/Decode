package org.firstinspires.ftc.teamcode.commands

import com.rathippo.commandviewer.CommandViewer

class WaitUntil(val f: () -> Boolean, name: String = "WaitUntil"): OverrideButtonCommand(name) {
    override fun getButtons(): ArrayList<Pair<Int, String>> {
        val buttons = super.getButtons()
        return buttons
    }
    override fun run(): CommandResult {
        return if (f()) {
            CommandResult.End(Result.success("End condition satisfied"))
        } else {
            CommandResult.Continue
        }
    }
}
fun WaitUntil(f: () -> Boolean) = WaitUntil(f, "WaitUntil")