#include "LidarDevice.h"
#include "cinder/Log.h"
#include <Windows.h>
#include "MiniConfig.h"

using namespace std;

void LidarDevice::info_(const string &err)
{
    CI_LOG_I(err);
    status = err;
}

static const float BlobCombinationThreshold = 0.05f * 1000;      // points' distance less than this value will be treat as same blob

#define PI       3.14159265358979323846

struct Point2D {
    float x, y;

    Point2D() : x(0), y(0) {}
    Point2D(float nx, float ny) : x(nx), y(ny) {}
};


struct ABlob {
    float min_x, min_y;
    float max_x, max_y;
    vector<Point2D> laserPoints;

    ABlob(float minX, float minY, float maxX, float maxY)
        : min_x(minX), min_y(minY), max_x(maxX), max_y(maxY)
    {
    }

    ABlob(float x, float y)
        : min_x(x), min_y(y), max_x(x), max_y(y)
    {
        laserPoints.push_back(Point2D(x, y));
    }

    void expand(const ABlob& other)
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

static ABlob CropArea(-0.75, -0.6, 0.75, 0.6);

static float blobContains(const ABlob& blob, float x, float y)
{
    return x >= blob.min_x && x <= blob.max_x && y >= blob.min_y && y <= blob.max_y;
}

static float blobDistance(const ABlob& a, const ABlob& b)
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

static int mergeBlobs(const list<ABlob>& input, list<ABlob>& output)
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
    float rad = (laserPoint.angle + BASE_ANGLE) * PI / 180.0f;
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

static void lookupBlobs(const LidarScan& scan, list<ABlob>& blobs)
{
    list<ABlob> tmp;

    for (auto laserPointIter = scan.begin(); laserPointIter != scan.end(); laserPointIter++)
    {
        auto& laserPoint = *laserPointIter;

        if (!laserPoint.valid)
            continue;

        float x, y;
        polar2cartesian(laserPoint, x, y);

        //if (!blobContains(CropArea, x, y))
            //continue;

        tmp.push_back(ABlob(x, y));
    }

#if 0
    swap(tmp, blobs);
#else
    while (true)
    {
        if (mergeBlobs(tmp, blobs) == 0)
            return;

        swap(tmp, blobs);
    }
#endif

}

//-----------------------------------------

LidarDevice::LidarDevice()
{
#if 0
    CropArea.min_x = driver_config.touch_area.left;
    CropArea.min_y = driver_config.touch_area.bottom;
    CropArea.max_x = driver_config.touch_area.right;
    CropArea.max_y = driver_config.touch_area.top;
#endif

    screenWidth = APP_WIDTH;
    screenWidth = APP_HEIGHT;
    s_touchPointList.clear();
}

vector<TouchPointStruct> LidarDevice::getTouchPoints()
{
    vector<TouchPointStruct> newTouchPointList;

    if (!scanData.empty())
    {
        list<ABlob> blobs;
        lookupBlobs(scanData, blobs);

        for (auto blobIter = blobs.begin(); blobIter != blobs.end(); blobIter++)
        {
            auto& blob = *blobIter;

            int x, y;
            physical2screen((blob.min_x + blob.max_x) / 2, (blob.max_y + blob.max_y) / 2, screenWidth, screenHeight, x, y);

            int len1, len2;

            if ((blob.max_x - blob.min_x) > (blob.max_y - blob.min_y))
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
         s_touchPointList.clear();
    }

    return s_touchPointList;
}
