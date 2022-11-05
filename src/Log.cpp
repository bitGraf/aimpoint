#include "Log.h"

#include <spdlog/fmt/ostr.h>
#include "spdlog/sinks/stdout_sinks.h"
#include "spdlog/sinks/stdout_color_sinks.h"

namespace aimpoint {

    std::shared_ptr<spdlog::logger> Log::s_Logger;

	void Log::Init(spdlog::level::level_enum base_level) {
        //spdlog::set_pattern("[%H:%M:%S] [%n] [%^%l%$] %v");
        spdlog::set_pattern("[%H:%M:%S] [%n] %^[%l] %v%$");

        // Create log instance
        s_Logger = spdlog::stdout_color_mt("LOGGERS");
        s_Logger->set_level(base_level);

        s_Logger->info("Logger created.");

        // Test levels
        s_Logger->trace("Trace Message!");
        s_Logger->debug("Debug Message!");
        s_Logger->info("Info Message!");
        s_Logger->warn("Warning Message!");
        s_Logger->error("Error Message!");
        s_Logger->critical("Critical Message!");
	}
}