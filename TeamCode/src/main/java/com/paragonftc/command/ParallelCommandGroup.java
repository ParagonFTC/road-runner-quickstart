package com.paragonftc.command;

import java.util.HashSet;

public class ParallelCommandGroup implements Command {
    private Command[] commands;
    private HashSet<Command> completedCommands;

    public ParallelCommandGroup(Command... commands) {
        this.commands = commands;
        completedCommands = new HashSet<Command>();
    }
    @Override
    public boolean isCompleted() {
        for (Command command : commands) {
            if (!command.isCompleted()) return false;
        }

        return true;
    }

    @Override
    public void start() {
        for (Command command : commands) {
            command.start();
        }
    }

    @Override
    public void update() {
        for (Command command : commands) {
            if (command.isCompleted() && !completedCommands.contains(command)) {
                command.end();
                completedCommands.add(command);
            } else {
                command.update();
            }
        }
    }

    @Override
    public void end() {
        for (Command command : commands) {
            if (!completedCommands.contains(command)) {
                command.end();
            }
        }
    }
}
