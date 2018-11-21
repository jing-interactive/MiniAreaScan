#include <Windows.h>
#include "nodeProcessDevice.h"

using namespace rp::standalone::rplidar;

using namespace rp::applet::rplidartouch;


static const float RplidarSampleDurationInUs = 476.4f;

static const float BlobCombinationThreshold = 0.05f;      // points' distance less than this value will be treat as same blob

static	ApplicationConfig driver_config;

#define PI       3.14159265358979323846

static rp::slamware::message::Message<LidarScan> lastScan;

static const int default_screen_width = 1920;
static const int default_screen_height = 1080;

static LARGE_INTEGER nFreq;
static LARGE_INTEGER nBeginTime;


NodeProcessDevice* _cdecl CreateObjectofDevice()
{
	return new NodeProcessDevice();
}

static void get_frequency_and_start_time()
{
	QueryPerformanceFrequency(&nFreq);
	QueryPerformanceCounter(&nBeginTime);
}

static long long get_time_in_ms()
{
    LARGE_INTEGER nCurrentTime;
	QueryPerformanceCounter(&nCurrentTime);

	return (nCurrentTime.QuadPart - nBeginTime.QuadPart) * 1000 / nFreq.QuadPart;
}

static void processScan(LidarScanPoint& output, const rplidar_response_measurement_node_t& input)
{
    if (!input.distance_q2)
    {
        output.valid = false;
        output.dist = 100000.0f;
    }
    else
    {
        output.valid = true;
        output.dist = input.distance_q2 / 4000.0f;
    }

    output.angle = (input.angle_q6_checkbit >> RPLIDAR_RESP_MEASUREMENT_ANGLE_SHIFT) / 64.0f;
}

static void processScan(rp::slamware::message::Message<LidarScan>& output, const rplidar_response_measurement_node_t* input, size_t scanSize)
{
    auto endTime = get_time_in_ms();
    auto startTime = endTime - scanSize * RplidarSampleDurationInUs / 1000.0f;

    output.timestamp = (rp::slamware::message::message_timestamp_t) startTime;

    output->resize(scanSize);

    for (size_t i = 0; i < scanSize; i++)
        processScan((*output)[i], input[i]);
}

struct Point2D {
    float x, y;
    
    Point2D() : x(0), y(0) {}
    Point2D(float nx, float ny) : x(nx), y(ny) {}
};


struct Blob {
    float min_x, min_y;
    float max_x, max_y;
    std::vector<Point2D> laserPoints;
    
    Blob(float minX, float minY, float maxX, float maxY)
        : min_x(minX), min_y(minY), max_x(maxX), max_y(maxY)
    {
    }
    
    Blob(float x, float y)
        : min_x(x), min_y(y), max_x(x), max_y(y)
    {
        laserPoints.push_back(Point2D(x, y));
    }
    
    void expand(const Blob& other)
    {
        if (other.min_x < min_x)
            min_x = other.min_x;
        
        if (other.max_x > max_x)
            max_x = other.max_x;
        
        if (other.min_y < min_y)
            min_y = other.min_y;
        
        if (other.max_y > max_y)
            max_y = other.max_y;
        
        for (auto iter = other.laserPoints.begin(); iter != other.laserPoints.end(); iter++)
        {
            auto& point = *iter;
            laserPoints.push_back(point);
        }
    }
    
    Point2D massCenter() const
    {
        if (laserPoints.empty())
            return Point2D((min_x + min_y) / 2, (max_x + max_y) / 2);
        
        if (laserPoints.size() == 1)
            return laserPoints[0];
        
        float sumX = 0;
        float sumY = 0;
        
        for (auto iter = laserPoints.begin(); iter != laserPoints.end(); iter++)
        {
            auto& pt = *iter;
            sumX += pt.x;
            sumY += pt.y;
        }
        
        return Point2D(sumX / laserPoints.size(), sumY / laserPoints.size());
    }
};

static Blob CropArea(-0.75, -0.6, 0.75, 0.6);

static float blobContains(const Blob& blob, float x, float y)
{
    return x >= blob.min_x && x <= blob.max_x && y >= blob.min_y && y <= blob.max_y;
}

static float blobDistance(const Blob& a, const Blob& b)
{
    if (blobContains(a, b.min_x, b.min_y))
        return 0;
    
    if (blobContains(a, b.max_x, b.min_y))
        return 0;
    
    if (blobContains(a, b.min_x, b.max_y))
        return 0;
    
    if (blobContains(a, b.max_x, b.max_y))
        return 0;
    
    float x_distance = a.max_x < b.min_x ? b.min_x - a.max_x : a.min_x - b.max_x;
    float y_distance = a.max_y < b.min_y ? b.min_y - a.max_y : a.min_y - b.max_y;
    
    return sqrtf(x_distance*x_distance + y_distance*y_distance);
}

static int mergeBlobs(const std::list<Blob>& input, std::list<Blob>& output)
{
    int mergedBlobs = 0;
    output.clear();
    
    for (auto iter = input.begin(); iter != input.end(); iter++)
    {
        auto& inputBlob = *iter;

        bool merged = false;
        
        for (auto compareBlobIter = output.begin(); compareBlobIter != output.end(); compareBlobIter++)
        {
            auto& compareBlob = *compareBlobIter;
            if (blobDistance(inputBlob, compareBlob) < BlobCombinationThreshold)
            {
                merged = true;
                compareBlob.expand(inputBlob);
                mergedBlobs++;
            }
        }
        
        if (!merged)
            output.push_back(inputBlob);
    }
    
    return mergedBlobs;
}

static void polar2cartesian(const LidarScanPoint& laserPoint, float& x, float& y)
{
	float rad = (laserPoint.angle + driver_config.angle_offset) * PI / 180.0f ;
    x = laserPoint.dist * -sin(rad);
    y = laserPoint.dist * cos(rad);
}

static void physical2screen(float x, float y, int screenWidth, int screenHeight, int& screenX, int& screenY)
{
	float xp = (x - CropArea.min_x) / (CropArea.max_y - CropArea.min_y);
    float yp = (y - CropArea.min_y) / (CropArea.max_y - CropArea.min_y);
    
    screenX = xp * screenHeight + (screenWidth - screenHeight * (CropArea.max_x - CropArea.min_x) / (CropArea.max_y - CropArea.min_y)) / 2;
    screenY = yp * screenHeight;
}

static void lookupBlobs(const LidarScan& scan, std::list<Blob>& blobs)
{
    std::list<Blob> tmp;

    for (auto laserPointIter = scan.begin(); laserPointIter != scan.end(); laserPointIter++)
    {
        auto& laserPoint = *laserPointIter;

        if (!laserPoint.valid)
            continue;
        
        float x, y;
        polar2cartesian(laserPoint, x, y);
        
        if (!blobContains(CropArea, x, y))
            continue;
        
        tmp.push_back(Blob(x, y));
    }
    
#if 0
    std::swap(tmp, blobs);
#else
    while (true)
    {
        if (mergeBlobs(tmp, blobs) == 0)
            return;
        
        std::swap(tmp, blobs);
    }
#endif
    
}

//-----------------------------------------

static std::list<TouchPointStruct> s_touchPointList;
static int timeLimit = 1000;

NodeProcessDevice::NodeProcessDevice() :lidarDevice_(nullptr)
{
	s_touchPointList.clear();
}

NodeProcessDevice::~NodeProcessDevice()
{
    clean();
}

bool NodeProcessDevice::createAndStartRPLidar(const char* path)
{
	char connectParm1[100];
	_u32 connectParm2;

	if(strncmp(path, "Serial:", 7) == 0)
	{
		lidarDevice_ = RPlidarDriver::CreateDriver(RPlidarDriver::DRIVER_TYPE_SERIALPORT);
		sscanf(path, "Serial:%[^,],%d", &connectParm1, &connectParm2);
	}
    else if (strncmp(path, "TCP:", 4) == 0)
	{
		lidarDevice_ = RPlidarDriver::CreateDriver(RPlidarDriver::DRIVER_TYPE_TCP);
		sscanf(path, "TCP:%[^,],%d", &connectParm1, &connectParm2);
	}

	if(!lidarDevice_)
	{
		rpos::system::config::LoggerInfoOut("Cannot create rplidar device");
		return false;
	}

	if(lidarDevice_->connect(connectParm1, connectParm2) != RESULT_OK)
	{
		rpos::system::config::LoggerInfoOut("Cannot connect to the rplidar device");
		return false;
	}

	rpos::system::config::LoggerInfoOut("Successfully create and connect to the rplidar device");
	
	get_frequency_and_start_time();
	
	lidarDevice_->setMotorPWM(600);
    lidarDevice_->startScan();

	return true;
}

void NodeProcessDevice::clean()
{
	if(lidarDevice_)
	{
		lidarDevice_->setMotorPWM(0);
		delete lidarDevice_;
		lidarDevice_ = nullptr;
	}
}

void NodeProcessDevice::startUp(const ApplicationConfig& config)
{
	driver_config = config;

	const char* connectPath = driver_config.connect_path;
	if(!createAndStartRPLidar(connectPath))
		return;

    CropArea.min_x = driver_config.touch_area.left;
    CropArea.min_y = driver_config.touch_area.bottom;
    CropArea.max_x = driver_config.touch_area.right;
    CropArea.max_y = driver_config.touch_area.top;

}

void NodeProcessDevice::setLidarPwm(int newPwm)
{
	if(lidarDevice_)
		lidarDevice_->setMotorPWM(newPwm);
}

bool NodeProcessDevice::getLidarScan(rp::slamware::message::Message<LidarScan>& scan)
{
    rplidar_response_measurement_node_t nodes[2048]; 
    size_t cnt = _countof(nodes);

    if (lidarDevice_->grabScanData(nodes, cnt, 0) == RESULT_OK)
    {
        rp::slamware::message::Message<LidarScan> scanTmp;
        processScan(scanTmp, nodes, cnt);
		scan = scanTmp;

		s_touchPointList.clear();
		return true;
    }
	return false;
}

std::list<TouchPointStruct> NodeProcessDevice::getTouchPoints(bool useDefaultScreenSize)
{		
    static auto lastTimeHasScan = get_time_in_ms();    //record the time that nodes has scaned for the last time

	std::list<TouchPointStruct> newTouchPointList;

	rp::slamware::message::Message<LidarScan> scan;
	if(getLidarScan(scan))
	{
		lastTimeHasScan = get_time_in_ms();

		if (lastScan.timestamp != scan.timestamp)
        {
            int freq = 1000 / (scan.timestamp - lastScan.timestamp);
            rpos::system::config::LoggerInfoOut("Scan frequency: %d", freq);
        }
        lastScan = scan;
        
        std::list<Blob> blobs;
        lookupBlobs(*scan, blobs);
		
		int screenWidth, screenHeight;
		if(useDefaultScreenSize)
		{
			screenWidth = default_screen_width;
			screenHeight = default_screen_height;
		}
		else
		{
			getScreenSize(screenWidth, screenHeight);
		}

		for (auto blobIter = blobs.begin(); blobIter != blobs.end(); blobIter++)
		{
			auto& blob = *blobIter;

            int x, y;
            physical2screen((blob.min_x + blob.max_x)/2, (blob.max_y + blob.max_y)/2, screenWidth, screenHeight, x, y);

			int len1, len2;

			if((blob.max_x - blob.min_x) > (blob.max_y - blob.min_y))
			{
				float xp1 = (blob.min_x - CropArea.min_x) / (CropArea.max_y - CropArea.min_y);
				len1 = xp1 * screenHeight + (screenWidth - screenHeight * (CropArea.max_x - CropArea.min_x) / (CropArea.max_y - CropArea.min_y)) / 2;

				float xp2 = (blob.max_x - CropArea.min_x) / (CropArea.max_y - CropArea.min_y);
				len2 = xp2 * screenHeight + (screenWidth - screenHeight * (CropArea.max_x - CropArea.min_x) / (CropArea.max_y - CropArea.min_y)) / 2;
			}
			else
			{
				float yp1 = (blob.min_y - CropArea.min_y) / (CropArea.max_y - CropArea.min_y);
				len1 = yp1 * screenHeight;

				float yp2 = (blob.max_y - CropArea.min_y) / (CropArea.max_y - CropArea.min_y);
				len2 = yp2 * screenHeight;
			}
			int radius = (len2 - len1) / 5;
			
			newTouchPointList.push_back(TouchPointStruct(x, y, radius));
		}
		s_touchPointList = newTouchPointList;
	}

	//clear s_touchPointList if time limits has past and nothing has scaned at all
	else
	{
		auto now = get_time_in_ms();
		if(now - lastTimeHasScan > timeLimit)
		{
			s_touchPointList.clear();
		}
	}

	return s_touchPointList;
}

std::list<ScanNodeDrawPoint> NodeProcessDevice::getScanNodeDrawPoints()
{
	std::list<ScanNodeDrawPoint> result;

	int screenWidth, screenHeight;
	getScreenSize(screenWidth, screenHeight);

	for (auto laserPointIter = lastScan->begin(); laserPointIter != lastScan->end(); laserPointIter++)
	{
		auto& laserPoint = *laserPointIter;

		if (!laserPoint.valid)
			continue;
        
		float x, y;
		polar2cartesian(laserPoint, x, y);

		if (!blobContains(CropArea, x, y))
			continue;
        
		int screenX, screenY;
		physical2screen(x, y, screenWidth, screenHeight, screenX, screenY);

		result.push_back(ScanNodeDrawPoint(screenX, screenY));
	}

	return result;
}

void NodeProcessDevice::setScreenSize(int width, int height)
{
	screenWidth_ = width;
	screenHeight_ = height;
}

void NodeProcessDevice::getScreenSize(int &width, int &height) const
{
	width = screenWidth_;
	height = screenHeight_;
}

