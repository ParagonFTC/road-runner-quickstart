package com.paragonftc.command;

/**
 * The Command interface is the framework for command-based design.
 * Each command represents a certain action.
 */
public interface Command {
    /**
     * Tells if the command is completed
     * @return if the command is completed
     */
    boolean isCompleted();

    /**
     * Initializes the command
     */
    void start();

    /**
     * Used to update stuff when the command has started but has not yet been completed yet
     */
    void update();

    /**
     * Is typically called when isCompleted() returns true.
     * Usually stops the actions that the command started
     */
    void end();
}
