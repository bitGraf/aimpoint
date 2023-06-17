#include "log.h"

#include <spdlog/spdlog.h>

static bool spdlog_initialized;

void load_log_file(const char* log_filename, log_level level) {
    spdlog::warn("Log file not supported yet!");
}

void set_terminal_log_level(log_level level) {
    if (!spdlog_initialized) {
        spdlog::set_pattern("%^[%l] %v%$");

        //spdlog::info("Logger format set.");

        //spdlog::info("------------------------");
        //spdlog::trace("Trace message");
        //spdlog::debug("Debug message");
        //spdlog::info("Info message");
        //spdlog::warn("Warning message");
        //spdlog::error("Error message");
        //spdlog::critical("Critial message");
        //spdlog::info("------------------------");

        spdlog_initialized = true;
    }

    spdlog::set_level(static_cast<spdlog::level::level_enum>(level));
}