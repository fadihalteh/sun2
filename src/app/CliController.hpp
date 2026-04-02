#pragma once

/**
 * @file CliController.hpp
 * @brief Simple CLI command handler for the headless runtime.
 *
 * Supported responsibilities:
 * - read text commands from stdin
 * - drive manual/auto transitions
 * - send explicit manual setpoints
 * - request clean shutdown
 *
 * This class is not the realtime control path.
 */

#include <string>

namespace solar {
class SystemManager;
}

namespace solar::app {

/**
 * @brief Headless CLI controller.
 */
class CliController {
public:
    /**
     * @brief Construct the CLI controller.
     *
     * @param system Runtime system manager controlled by this CLI.
     */
    explicit CliController(SystemManager& system);

    CliController(const CliController&) = delete;
    CliController& operator=(const CliController&) = delete;

    /**
     * @brief Attach a file descriptor used for CLI text input.
     *
     * @param fd File descriptor supplying readable CLI text.
     */
    void attachInputFd(int fd);

    /**
     * @brief Detach the currently attached input file descriptor.
     */
    void detachInputFd();

    /**
     * @brief Handle one periodic loop tick.
     *
     * @return False if the application should terminate.
     */
    bool onTick();

    /**
     * @brief Handle readable input on the attached file descriptor.
     *
     * @return False if the application should terminate.
     */
    bool onInputReady();

private:
    bool executeLine_(const std::string& line);
    void printHelp_() const;

private:
    SystemManager& system_;
    int input_fd_{-1};
    std::string pending_;
};

} // namespace solar::app
