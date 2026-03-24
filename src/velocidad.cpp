
#include <opencv2/opencv.hpp>
#include <iostream>

float pixel_to_meters(float pixel_displacement, float Z_meters)
{
    const float fx = 620.0f; // AJUSTADO EMPÍRICAMENTE

    return (pixel_displacement * Z_meters) / fx;
}

float pixel_to_meters_viejo(float pixel_displacement, float Z_meters)
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
    //double meters = (pixel_displacement / Z_meters) * 0.03; // 0.03 mts (3cm) por pixel a 1 metro

    return meters;
}

// ===============================
// FUNCIÓN: desplazamiento horizontal con LK
// ===============================
float desplazamientoHorizontalLK(const cv::Mat& img1, const cv::Mat& img2, int x1, int x2,
                                 float roiWidthRatio = 0.7)
                                 //double roiWidthRatio = 0.05)
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

    x0 = 200;
    roi_width = 200;

    cv::Rect roi(x0, 0, roi_width, h);

    cv::Mat r1 = img1(roi);
    cv::Mat r2 = img2(roi);

    std::vector<cv::Point2f> p0, p1;
    //cv::goodFeaturesToTrack(r1, p0, 200, 0.01, 10);
    cv::goodFeaturesToTrack(r1, p0, 2000, 0.01, 5);

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


    //NUEVO

    std::vector<float> dx_valid;

for (size_t i = 0; i < p0.size(); i++) {
    if (!status[i]) continue;

    cv::Point2f d = p1[i] - p0[i];

    // quedarte solo con movimiento hacia la derecha y significativo
    if (d.x > 1.0f) {  // <-- ajustá este valor
        dx_valid.push_back(d.x);
    }
}

// validar cantidad mínima
if (dx_valid.size() < 5)
    return 0.0f;

// mediana robusta
size_t mid = dx_valid.size() / 2;
std::nth_element(dx_valid.begin(), dx_valid.begin() + mid, dx_valid.end());

return dx_valid[mid];

    // NUEVO
    // --- construir vectores de movimiento ---
std::vector<std::pair<float, cv::Point2f>> mags;

for (size_t i = 0; i < p0.size(); i++) {
    if (!status[i]) continue;

    cv::Point2f d = p1[i] - p0[i];
    float mag = cv::norm(d);

    mags.push_back({mag, d});
}

if (mags.empty())
    return 0.0f;

// --- ordenar por magnitud (descendente) ---
std::sort(mags.begin(), mags.end(),
          [](const auto& a, const auto& b) { return a.first > b.first; });

// --- quedarse con top 20% ---
//int N = std::max(1, (int)(mags.size() * 0.2f));
int N = std::max(1, (int)(mags.size() * 0.8f));
std::vector<cv::Point2f> candidatos;

for (int i = 0; i < N; i++)
    candidatos.push_back(mags[i].second);

// --- promedio para coherencia ---
cv::Point2f mean(0, 0);
for (auto& d : candidatos)
    mean += d;

mean *= (1.0f / candidatos.size());

// --- filtrar por coherencia ---
std::vector<cv::Point2f> finales;

for (auto& d : candidatos) {
    //if (cv::norm(d - mean) < 2.0f) // <-- ajustá este valor
    if (cv::norm(d - mean) < 8.0f) // <-- ajustá este valor
        finales.push_back(d);
}

if (finales.empty())
    return 0.0f;

// --- mediana (robusto a outliers) ---
std::vector<float> dx, dy;

for (auto& d : finales) {
    dx.push_back(d.x);
    dy.push_back(d.y);
}

//size_t mid = dx.size() / 2;

std::nth_element(dx.begin(), dx.begin() + mid, dx.end());
std::nth_element(dy.begin(), dy.begin() + mid, dy.end());

cv::Point2f movimiento(dx[mid], dy[mid]);

// ejemplo: devolver magnitud final
return movimiento.x;

















    // ANTES
    float sum_dx = 0.0f;
    int count = 0;

    /*
    for (size_t i = 0; i < p0.size(); i++)
    {
        if (status[i])
        {
            double dx = p1[i].x - p0[i].x;
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

// USANDO LA MEDIANA
//std::sort(dxs.begin(), dxs.end());
//float dx_med = dxs[dxs.size()/2];

//float incremento_hack = 0.5*dx_med - 20.0;
//dx_med += incremento_hack;
//return dx_med;


// 🔽 Ordenar de mayor a menor
std::sort(dxs.begin(), dxs.end(), std::greater<float>());

// 🔽 Tomar los 10 mejores
//int n = std::min(10, (int)dxs.size());
int n = std::min(10, (int)dxs.size());
//

sum_dx = 0.0f;
for (int i = 0; i < n; i++)
{
   sum_dx += dxs[i];
}

count = n;

  return sum_dx / count;
}

