
#include <opencv2/opencv.hpp>
#include <iostream>


float pixel_to_meters(float pixel_displacement, float Z_meters)
{
    // Parámetro de calibración original (640x480)
    const float fx_original = 820.2028f;
    const float width_original = 640.0f;

    // Resolución real actual (la que te devuelve la cámara)
    const float width_current = 864.0f;

    // Escalado de la focal
    float scale_x = width_current / width_original;
    float fx = fx_original * scale_x;

    // Conversión píxeles -> metros
    float meters = (pixel_displacement * Z_meters) / fx;
    //float meters = (pixel_displacement / Z_meters) * 0.03; // 0.03 mts (3cm) por pixel a 1 metro

    return meters;
}

// ===============================
// FUNCIÓN: desplazamiento horizontal con LK
// ===============================
float desplazamientoHorizontalLK(const cv::Mat& img1, const cv::Mat& img2, int x1, int x2,
                                 float roiWidthRatio = 0.7)
                                 //float roiWidthRatio = 0.05)
{
    CV_Assert(img1.type() == CV_8UC1 && img2.type() == CV_8UC1);
    CV_Assert(img1.size() == img2.size());

    int w = img1.cols;
    int h = img1.rows;

    int roi_width = w * roiWidthRatio;
    int x0 = (w - roi_width) / 2;
    x0 = x1;
    roi_width = x2;
    if (roi_width<100)
    	roi_width = 100;

    cv::Rect roi(x0, 0, roi_width, h);

    cv::Mat r1 = img1(roi);
    cv::Mat r2 = img2(roi);

    std::vector<cv::Point2f> p0, p1;
    cv::goodFeaturesToTrack(r1, p0, 200, 0.01, 10);

    if (p0.empty())
        return 0.0f;

    std::vector<unsigned char> status;
    std::vector<float> err;

    cv::calcOpticalFlowPyrLK(
        r1, r2,
        p0, p1,
        status, err,
        cv::Size(21,21), 3,
        cv::TermCriteria(cv::TermCriteria::COUNT|cv::TermCriteria::EPS, 30, 0.01)
    );

    float sum_dx = 0.0f;
    int count = 0;

    /*
    for (size_t i = 0; i < p0.size(); i++)
    {
        if (status[i])
        {
            float dx = p1[i].x - p0[i].x;
            sum_dx += dx;
            count++;
        }
    }

    if (count == 0)
        return 0.0f;

    */
    // 🔽 Guardar solo dx positivos
std::vector<float> dxs;

for (size_t i = 0; i < p0.size(); i++)
{
    if (status[i])
    {
        float dx = p1[i].x - p0[i].x;

        if (dx > 0) // solo hacia la derecha
            dxs.push_back(dx);
    }
}

// Si no hay puntos válidos
if (dxs.empty())
    return 0.0f;

// 🔽 Ordenar de mayor a menor
std::sort(dxs.begin(), dxs.end(), std::greater<float>());

// 🔽 Tomar los 10 mejores
//int n = std::min(10, (int)dxs.size());
int n = std::min(3, (int)dxs.size());

sum_dx = 0.0f;
for (int i = 0; i < n; i++)
{
    sum_dx += dxs[i];
}

count = n;

    return sum_dx / count;
}

