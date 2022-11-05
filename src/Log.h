#pragma once

#include <spdlog/spdlog.h>

namespace aimpoint {

	class Log {
	public:
		static void Init(spdlog::level::level_enum base_level = spdlog::level::trace);

		inline static std::shared_ptr<spdlog::logger>& GetLogger() { return s_Logger; }

	private:
		static std::shared_ptr<spdlog::logger> s_Logger;
	};

}

// Core Logging Macros
/*
* trace    = SPDLOG_LEVEL_TRACE,
* debug    = SPDLOG_LEVEL_DEBUG,
* info     = SPDLOG_LEVEL_INFO,
* warn     = SPDLOG_LEVEL_WARN,
* err      = SPDLOG_LEVEL_ERROR,
* critical = SPDLOG_LEVEL_CRITICAL,
* off      = SPDLOG_LEVEL_OFF,
*/
#define LOG_TRACE(...)	aimpoint::Log::GetLogger()->trace(__VA_ARGS__)
#define LOG_DEBUG(...)	aimpoint::Log::GetLogger()->debug(__VA_ARGS__)
#define LOG_INFO(...)	aimpoint::Log::GetLogger()->info(__VA_ARGS__)
#define LOG_WARN(...)	aimpoint::Log::GetLogger()->warn(__VA_ARGS__)
#define LOG_ERROR(...)	aimpoint::Log::GetLogger()->error(__VA_ARGS__)
#define LOG_FATAL(...)	aimpoint::Log::GetLogger()->critical(__VA_ARGS__)