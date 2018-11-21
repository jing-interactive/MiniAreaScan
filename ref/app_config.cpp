#include "app_config.h"
#include <fstream>

using namespace rp::applet::rplidartouch;

namespace rpos { namespace system { namespace config {

	#define RPOS_CORE_LOG_MAX_SINGLE_LINE   65536

	static Json::Value root;  

	bool ReadJsonFromFile(const char* filename, rp::applet::rplidartouch::ApplicationConfig& that)  
    {  
        Json::Reader reader;

        std::ifstream is;  
        is.open (filename, std::ios::binary );  

        if (!reader.parse(is, root, false))
            return false;


        int int_tmp = root.get("show_lidar_scan", "null").asInt();
		that.show_lidar_scan = (int_tmp == 1) ? true: false;

		int_tmp = root.get("show_touch_point_coordination", "null").asInt();
		that.show_touch_point_coordination = (int_tmp == 1) ? true: false;
            
		that.angle_offset = root.get("angle_offset", "null").asDouble();

		std::string ss = root.get("connect_path", "null").asString();
		const char* chrTmpPath = ss.c_str();
		memset(that.connect_path, 0, sizeof(that.connect_path));
		memcpy(that.connect_path, chrTmpPath, strlen(chrTmpPath));

		for(int i = 0; i < root["touch_area"].size(); ++i)
		{
			Json::Value float_tmp;
			float_tmp = root["touch_area"][i];
			switch (i)
			{
			case 0:
				that.touch_area.left = float_tmp.asDouble();
				break;
			case 1:
				that.touch_area.bottom  = float_tmp.asDouble();
				break;
			case 2:
				that.touch_area.right = float_tmp.asDouble();
				break;
			case 3:
				that.touch_area.top  = float_tmp.asDouble();
				break;
			}
		}
        is.close();  
		
        return true;  
    } 

	bool WriteCropAreaToFile(const char* filename, const rp::applet::rplidartouch::RecTouch &rec)
	{
		Json::StyledStreamWriter writer;
		std::ofstream fileWrite(filename);
		float size[4] = {rec.left, rec.bottom, rec.right, rec.top};

		for(int i = 0; i < root["touch_area"].size(); ++i)
		{
			root["touch_area"][i] = size[i];
		}
		writer.write(fileWrite, root);
		return true;
	}

	void LoggerInfoOut(const char* msg, ...)
	{
		va_list args;
        va_start(args, msg);
		char buffer[RPOS_CORE_LOG_MAX_SINGLE_LINE];
		vsnprintf(buffer, RPOS_CORE_LOG_MAX_SINGLE_LINE, msg, args);
		buffer[RPOS_CORE_LOG_MAX_SINGLE_LINE - 1] = 0;
        va_end(args);
		std::cout << buffer << std::endl;
	}

} } }
