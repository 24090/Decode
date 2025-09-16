package org.firstinspires.ftc.teamcode.commands

import com.rathippo.commandviewer.CommandViewer

class Forever(val f: () -> Command, name: String = "Forever"): OverrideButtonCommand(name) {
    val pastCommands: ArrayList<DeadCommand> = arrayListOf()
    val clearDead = CommandViewer.registerFunction { pastCommands.clear() }
    var currentCommand: Command? = f.invoke()
    override fun getButtons(): ArrayList<Pair<Int, String>> {
        val buttons = super.getButtons()
        buttons.add(Pair(clearDead, "Clear Dead Commands"))
        return buttons
    }
    override fun run(): CommandResult {
        for (command in pastCommands){
            command.update()
        }
        when (val result = currentCommand!!.update()) {
            CommandResult.Continue -> {}
            is CommandResult.End -> {
                pastCommands.add(currentCommand!!.getDead(result))
                if (result.result.isFailure) {
                    currentCommand = null
                    return result
                }
                currentCommand = f.invoke()
            }
        }
        return CommandResult.Continue
    }

    override fun getDead(reason: CommandResult.End): DeadCommand {
        currentCommand?.let {
            pastCommands.add(it.getDead(CommandResult.End(Result.failure(Error("Parent ended early")))))
        }
        return DeadCommand(reason, pastCommands, name, selfCondense, uid)
    }

    override fun getFuture(): FutureCommand {
        val commands = pastCommands.clone() as ArrayList<Command>
        currentCommand?.let {
            commands.add(it.getFuture())
        }
        return FutureCommand(commands, name, selfCondense, uid)
    }
}