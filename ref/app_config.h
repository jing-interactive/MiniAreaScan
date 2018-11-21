#pragma once

#include <json/json.h>
#include <stdarg.h>

namespace rp { namespace applet { namespace rplidartouch {

	struct RecTouch
	{
		float top;
		float bottom;
		float left;
		float right;

		RecTouch(): top(0.0f), bottom(0.0f), left(0.0f), right(0.0f){}
		RecTouch(float n_left, float n_botoom, float n_right, float n_top): left(n_left), bottom(n_botoom), right(n_right), top(n_top) {}
	};
	
	struct ApplicationConfig {
		char connect_path[100];
		RecTouch touch_area;
		bool show_lidar_scan;
		bool show_touch_point_coordination;
		double angle_offset;
	};

} } }

namespace rpos { namespace system { namespace config {

	extern "C" _declspec(dllexport) bool ReadJsonFromFile(const char* filename, rp::applet::rplidartouch::ApplicationConfig& that);
	extern "C" _declspec(dllexport) bool WriteCropAreaToFile(const char* filename, const rp::applet::rplidartouch::RecTouch &rec);

	extern "C" _declspec(dllexport) void LoggerInfoOut(const char* msg, ...);

} } }
