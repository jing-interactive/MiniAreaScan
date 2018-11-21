#pragma once

#include "message.h"
#include "app_config.h"
#include "rplidar.h"
#include <list>

#define MULTITOUCH_SDK_API extern "C" __declspec(dllexport)

struct LidarScanPoint
{
    float dist;     // in meter
    float angle;    // in degree, 0 expected to be the front of LIDAR, and increase by rotate in counter-clockwise (left-hand system)
    bool valid;     // if the lidar scan point is valid or not (for eg. no obstacle detected)
};

typedef std::vector<LidarScanPoint> LidarScan;

struct TouchPointStruct
{
	int x;
	int y;
	float radius;

	TouchPointStruct() :x(0), y(0), radius(0.0f){}
	TouchPointStruct(int n_x, int n_y, float n_radius) :x(n_x), y(n_y), radius(n_radius){}
};

struct ScanNodeDrawPoint
{
	int x;
	int y;

	ScanNodeDrawPoint() :x(0), y(0) {}
	ScanNodeDrawPoint(int n_x, int n_y) :x(n_x), y(n_y) {}
};

class NodeProcessDevice
{
public:
    NodeProcessDevice();
    virtual ~NodeProcessDevice();
	
	virtual bool createAndStartRPLidar(const char* path);
	virtual void clean();

	virtual void startUp(const rp::applet::rplidartouch::ApplicationConfig& config);


	virtual void setLidarPwm(int newPwm);
	virtual void setScreenSize(int width, int height);

	virtual std::list<TouchPointStruct> getTouchPoints(bool useDefaultScreenSize = false);
	virtual std::list<ScanNodeDrawPoint> getScanNodeDrawPoints();
    virtual bool getLidarScan(rp::slamware::message::Message<LidarScan>& scan);

private:
	void getScreenSize(int &width, int &height) const;

private:
    rp::standalone::rplidar::RPlidarDriver *lidarDevice_;
	int screenWidth_;
	int screenHeight_;
};

MULTITOUCH_SDK_API NodeProcessDevice* _cdecl CreateObjectofDevice();