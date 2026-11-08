// ============================================================

//  工业相机 + YOLO 实时推理（精简版）

//  流程: 相机连接 -> 采集 -> 调参 -> 格式转换 -> 推理 -> 显示

//

//  编译（CLion 用 CMakeLists.txt；命令行用下面这条）:

//    g++ -O2 -std=c++17 mvs_yolo.cpp -o mvs_yolo \

//      $(pkg-config --cflags opencv4) -I/opt/MVS/include \

//      -L/opt/MVS/lib/64 -lMvCameraControl \

//      $(pkg-config --libs opencv4) -Wl,-rpath,/opt/MVS/lib/64

//

//  用法:

//    ./mvs_yolo --list                       列出相机

//    ./mvs_yolo --model m.onnx --image a.jpg 用图片测推理(不用相机)

//    ./mvs_yolo --model m.onnx --classes B1,B2,B3,R1,R2,R3   实时

// ============================================================

#include <MvCameraControl.h>

#include <opencv2/opencv.hpp>

#include <algorithm>

#include <chrono>

#include <cmath>

#include <cstddef>

#include <iostream>

#include <sstream>

#include <stdexcept>

#include <string>

#include <utility>

#include <vector>

// ---------- 工具 ----------

static std::string MvName(unsigned int c) {

    switch (c) {

    case MV_OK:          return "成功";

    case MV_E_HANDLE:    return "句柄无效";

    case MV_E_NODATA:    return "无数据(超时)";

    case MV_E_CALLORDER: return "调用顺序错";

    case MV_E_PARAMETER: return "参数错误";

    case MV_E_UNKNOW:    return "未知错误(设备被占用?)";

    default:             return "错误码";

    }

}

static void MvOk(int r, const char* what) {

    if (r == MV_OK) return;

    std::ostringstream o;

    o << "MVS失败[" << what << "]: 0x" << std::hex << (unsigned)r

      << " " << MvName((unsigned)r);

    throw std::runtime_error(o.str());

}

#define MVOK(x) MvOk((x), #x)

// SDK 的定长 char 数组 -> std::string（不保证有 \0）

static std::string S(const char* p, size_t n) {

    if (!p || !n) return "";

    size_t i = 0;

    while (i < n && p[i]) ++i;

    return std::string(p, i);

}

// 新增：支持 unsigned char 数组的重载
static std::string S(const unsigned char* p, size_t n) {
    return S(reinterpret_cast<const char*>(p), n);
}

template <size_t N> static std::string S(const char (&a)[N]) { return S(a, N); }
template <size_t N> static std::string S(const unsigned char (&a)[N]) { return S(a, N); }

// ---------- 相机 ----------

class Cam {

public:

    ~Cam() {

        try {

            if (grabbing_) { MV_CC_StopGrabbing(h_); grabbing_ = false; }

            if (opened_)   { MV_CC_CloseDevice(h_);   opened_ = false; }

            if (h_)        { MV_CC_DestroyHandle(h_); h_ = nullptr; }

            if (inited_)   { MV_CC_Finalize();        inited_ = false; }

        } catch (...) {}

    }

    // 枚举: 返回 "型号 | 序列号" 列表

    static std::vector<std::pair<std::string, std::string>> List() {

        std::vector<std::pair<std::string, std::string>> v;

        MVOK(MV_CC_Initialize());

        MV_CC_DEVICE_INFO_LIST ls{};

        MVOK(MV_CC_EnumDevices(MV_USB_DEVICE | MV_GIGE_DEVICE, &ls));

        for (unsigned i = 0; i < ls.nDeviceNum; ++i) {

            auto* d = ls.pDeviceInfo[i];

            if (!d) continue;

            if (d->nTLayerType == MV_USB_DEVICE) {

                v.push_back({S(d->SpecialInfo.stUsb3VInfo.chModelName,sizeof(d->SpecialInfo.stUsb3VInfo.chModelName)),
             S(d->SpecialInfo.stUsb3VInfo.chSerialNumber, sizeof(d->SpecialInfo.stUsb3VInfo.chSerialNumber))});

            } else if (d->nTLayerType == MV_GIGE_DEVICE) {

                v.push_back({S(d->SpecialInfo.stGigEInfo.chModelName,sizeof(d->SpecialInfo.stGigEInfo.chModelName)),

                             S(d->SpecialInfo.stGigEInfo.chSerialNumber,sizeof(d->SpecialInfo.stGigEInfo.chSerialNumber))});

            }

        }

        MV_CC_Finalize();

        return v;

    }

    void Open(const std::string& sn) {

        MVOK(MV_CC_Initialize()); inited_ = true;

        MV_CC_DEVICE_INFO_LIST ls{};

        MVOK(MV_CC_EnumDevices(MV_USB_DEVICE | MV_GIGE_DEVICE, &ls));

        MV_CC_DEVICE_INFO* sel = nullptr;

        for (unsigned i = 0; i < ls.nDeviceNum; ++i) {

            auto* d = ls.pDeviceInfo[i];

            if (!d) continue;

            std::string s;

            if (d->nTLayerType == MV_USB_DEVICE)

                s = S(d->SpecialInfo.stUsb3VInfo.chSerialNumber,sizeof(d->SpecialInfo.stUsb3VInfo.chSerialNumber));

            else if (d->nTLayerType == MV_GIGE_DEVICE)

                s = S(d->SpecialInfo.stGigEInfo.chSerialNumber,sizeof(d->SpecialInfo.stGigEInfo.chSerialNumber));

            if (sn.empty() || s == sn) { sel = d; break; }

        }

        if (!sel) throw std::runtime_error(

            "找不到相机。检查: 1)USB3.0插好 2)系统识别(lsusb) "

            "3)MVS客户端已关闭 4)权限 plugdev");

        MVOK(MV_CC_CreateHandle(&h_, sel));

        MVOK(MV_CC_OpenDevice(h_));

        opened_ = true;

        std::cout << "[相机] 已打开 " << sn << "\n";

    }
    // 读曝光/增益范围（用于滑动条映射）

    void Range(float& emin, float& emax, float& gmin, float& gmax) {

        MVCC_FLOATVALUE v{};

        emin = emax = gmin = gmax = 0;

        if (MV_CC_GetFloatValue(h_, "ExposureTime", &v) == MV_OK) { emin = v.fMin; emax = v.fMax; }

        if (MV_CC_GetFloatValue(h_, "Gain", &v) == MV_OK)         { gmin = v.fMin; gmax = v.fMax; }

    }

    void Config(float exp, float gain) {

        MVOK(MV_CC_SetEnumValueByString(h_, "AcquisitionMode", "Continuous"));

        MVOK(MV_CC_SetEnumValueByString(h_, "TriggerMode", "Off"));

        MVOK(MV_CC_SetEnumValueByString(h_, "ExposureAuto", "Off"));

        MVOK(MV_CC_SetEnumValueByString(h_, "GainAuto", "Off"));

        MV_CC_SetEnumValueByString(h_, "ExposureMode", "Timed");  // 部分型号无此节点,忽略失败

        MVOK(MV_CC_SetEnumValue(h_, "PixelFormat", PixelType_Gvsp_BayerRG8));

        MVCC_FLOATVALUE v{};

        MVOK(MV_CC_GetFloatValue(h_, "ExposureTime", &v));

        if (exp < v.fMin || exp > v.fMax) {

            std::ostringstream o; o << "曝光 " << exp << " 超出 [" << v.fMin << "," << v.fMax << "]";

            throw std::runtime_error(o.str());

        }

        MVOK(MV_CC_SetFloatValue(h_, "ExposureTime", exp));

        MVOK(MV_CC_GetFloatValue(h_, "Gain", &v));

        if (gain >= v.fMin && gain <= v.fMax) MV_CC_SetFloatValue(h_, "Gain", gain);

        std::cout << "[相机] 曝光 " << exp << "us 增益 " << gain << "dB\n";

    }

    void SetExp(float us)  { if (h_) MV_CC_SetFloatValue(h_, "ExposureTime", us); }

    void SetGain(float db) { if (h_) MV_CC_SetFloatValue(h_, "Gain", db); }

    void Start() { if (!grabbing_) { MVOK(MV_CC_StartGrabbing(h_)); grabbing_ = true; } }

    void Stop()  { if (grabbing_ && h_) { MV_CC_StopGrabbing(h_); grabbing_ = false; } }

      // 取一帧 -> BGR。返回 false 表示超时

    bool Grab(cv::Mat& bgr, int timeoutMs) {

        MV_FRAME_OUT f{};

        int r = MV_CC_GetImageBuffer(h_, &f, timeoutMs);

        if ((unsigned)r == MV_E_NODATA) return false;

        MVOK(r);

        struct G { void* h; MV_FRAME_OUT* f; ~G() { MV_CC_FreeImageBuffer(h, f); } } g{h_, &f};

        const auto& in = f.stFrameInfo;

        const int w = (int)in.nWidth, hgt = (int)in.nHeight;

        if (!f.pBufAddr || w <= 0 || hgt <= 0) return false;

        const unsigned t = in.enPixelType;

        const bool mono = (t == PixelType_Gvsp_Mono8);

        const bool bgr8 = (t == PixelType_Gvsp_BGR8_Packed);

        const bool rgb8 = (t == PixelType_Gvsp_RGB8_Packed);

        const bool bayer = (t == PixelType_Gvsp_BayerRG8 || t == PixelType_Gvsp_BayerGB8 ||

                            t == PixelType_Gvsp_BayerGR8 || t == PixelType_Gvsp_BayerBG8);

        if (!mono && !bgr8 && !rgb8 && !bayer) {

            std::ostringstream o;

            o << "不支持的像素格式 0x" << std::hex << t << "，请改用 BayerRG8/BGR8/Mono8";

            throw std::runtime_error(o.str());

        }

        const size_t bpp = (bgr8 || rgb8) ? 3 : 1;

        size_t step = (size_t)w * bpp;

        if (in.nFrameLen >= step * (size_t)hgt) {         // 有行填充时用真实步长

            size_t c = in.nFrameLen / (size_t)hgt;

            if (c >= step) step = c;

        }

        if (bgr8) { bgr = cv::Mat(hgt, w, CV_8UC3, f.pBufAddr, step); return true; }

        if (rgb8) {

            cv::Mat v(hgt, w, CV_8UC3, f.pBufAddr, step);

            cv::cvtColor(v, bgr, cv::COLOR_RGB2BGR); return true;

        }

        if (mono) {

            cv::Mat v(hgt, w, CV_8UC1, f.pBufAddr, step);

            cv::cvtColor(v, bgr, cv::COLOR_GRAY2BGR); return true;

        }

        // Bayer -> BGR（用 OpenCV 转换，代码简洁）

        cv::Mat v(hgt, w, CV_8UC1, f.pBufAddr, step);

        int code = cv::COLOR_BayerRG2BGR;

        if (t == PixelType_Gvsp_BayerGB8) code = cv::COLOR_BayerGB2BGR;

        else if (t == PixelType_Gvsp_BayerGR8) code = cv::COLOR_BayerGR2BGR;

        else if (t == PixelType_Gvsp_BayerBG8) code = cv::COLOR_BayerBG2BGR;

        cv::cvtColor(v, bgr, code);

        return true;

    }

private:

    void* h_ = nullptr;

    bool inited_ = false, opened_ = false, grabbing_ = false;

};
// ---------- 推理 ----------

struct Det {

    cv::Rect2f box;

    int id = -1;

    float conf = 0;

    std::string name;

    std::vector<cv::Point2f> kp;

};

struct Opt {

    float conf = 0.25f, iou = 0.45f;

    int size = 640;

    bool letterbox = true;

    int numKpt = 4, kptDims = 2;

    std::vector<std::string> names;

};

class Detector {

public:

    void Load(const std::string& p) {

        net_ = cv::dnn::readNetFromONNX(p);

        if (net_.empty()) throw std::runtime_error("ONNX 加载失败: " + p);

        net_.setPreferableBackend(cv::dnn::DNN_BACKEND_OPENCV);

        net_.setPreferableTarget(cv::dnn::DNN_TARGET_CPU);

        std::cout << "[推理] 已加载 " << p << "\n";

    }

    std::vector<Det> Run(const cv::Mat& bgr, const Opt& o) {

        // --- letterbox 预处理 ---

        float sx, sy, px, py;

        cv::Mat in;

        if (o.letterbox) {

            float s = std::min((float)o.size / bgr.cols, (float)o.size / bgr.rows);

            int nw = (int)std::lround(bgr.cols * s), nh = (int)std::lround(bgr.rows * s);

            cv::Mat t; cv::resize(bgr, t, cv::Size(nw, nh));

            int dw = o.size - nw, dh = o.size - nh;

            px = dw / 2.f; py = dh / 2.f; sx = sy = s;

            cv::copyMakeBorder(t, in, dh / 2, dh - dh / 2, dw / 2, dw - dw / 2,

                               cv::BORDER_CONSTANT, cv::Scalar(114, 114, 114));

        } else {

            cv::resize(bgr, in, cv::Size(o.size, o.size));

            sx = (float)o.size / bgr.cols; sy = (float)o.size / bgr.rows; px = py = 0;

        }

        cv::Mat blob;

        // swapRB=true 必须: 相机给 BGR，模型要 RGB

        cv::dnn::blobFromImage(in, blob, 1 / 255.0, cv::Size(o.size, o.size),

                               cv::Scalar(), true, false, CV_32F);

        net_.setInput(blob);

        cv::Mat out = net_.forward();

        // out 通常是 (1,C,N) 或 (1,N,C)，统一成 (C,N)

        cv::Mat m = out.clone();

        if (m.dims == 3) m = m.reshape(1, m.size[1]);

        if (m.rows > m.cols) { cv::Mat t; cv::transpose(m, t); m = t; }  // 不能就地转置

        const int C = m.rows, N = m.cols;

        // 类别数：优先用用户给的类别名个数（最可靠）

        const int kv = o.numKpt * o.kptDims;

        int nc = o.names.empty() ? (C - 4 - kv) : (int)o.names.size();

        if (!o.names.empty() && 4 + nc + kv != C) {

            std::ostringstream e;

            e << "模型输出 C=" << C << "，但 4+" << nc << "+" << kv << "=" << (4 + nc + kv)

              << "。请检查 --classes 个数和关键点设置";

            // detect 模式（没有关键点）时不算错

            if (kv > 0) std::cerr << "[警告] " << e.str() << "\n";

        }

        if (nc <= 0) nc = C - 4;   // 兜底

        if (nc <= 0) throw std::runtime_error("无法推导类别数，请用 --classes 指定");

        const float* d = m.ptr<float>(0);

        std::vector<cv::Rect2f> bs;

        std::vector<float> sc;

        std::vector<int> ids;

        std::vector<std::vector<cv::Point2f>> kps;

        for (int i = 0; i < N; ++i) {

            int bi = -1; float bv = 0;

            for (int c = 0; c < nc; ++c) {

                float v = d[(4 + c) * N + i];

                if (v > bv) { bv = v; bi = c; }

            }

            if (bv < o.conf || bi < 0) continue;

            float cx = d[0 * N + i], cy = d[1 * N + i];

            float bw = d[2 * N + i], bh = d[3 * N + i];

            float x1 = (cx - bw / 2 - px) / sx, y1 = (cy - bh / 2 - py) / sy;

            float x2 = (cx + bw / 2 - px) / sx, y2 = (cy + bh / 2 - py) / sy;

            x1 = std::max(0.f, std::min(x1, (float)bgr.cols - 1));

            y1 = std::max(0.f, std::min(y1, (float)bgr.rows - 1));

            x2 = std::max(0.f, std::min(x2, (float)bgr.cols - 1));

            y2 = std::max(0.f, std::min(y2, (float)bgr.rows - 1));

            if (x2 <= x1 || y2 <= y1) continue;

            bs.emplace_back(x1, y1, x2 - x1, y2 - y1);

            sc.push_back(bv); ids.push_back(bi);

            std::vector<cv::Point2f> kp;

            int ks = 4 + nc;

            if (kv > 0 && C - ks >= kv) {

                for (int k = 0; k < o.numKpt; ++k) {

                    float kx = d[(ks + k * o.kptDims + 0) * N + i];

                    float ky = d[(ks + k * o.kptDims + 1) * N + i];

                    kp.emplace_back((kx - px) / sx, (ky - py) / sy);

                }

            }

            kps.push_back(kp);

        }

        // --- NMS（自己实现，支持浮点框）---

        std::vector<int> idx(bs.size());

        for (size_t i = 0; i < idx.size(); ++i) idx[i] = (int)i;

        std::sort(idx.begin(), idx.end(), [&](int a, int b) { return sc[a] > sc[b]; });

        std::vector<char> dead(bs.size(), 0);

        std::vector<int> keep;

        for (size_t i = 0; i < idx.size(); ++i) {

            if (dead[idx[i]]) continue;

            keep.push_back(idx[i]);

            for (size_t j = i + 1; j < idx.size(); ++j) {

                if (dead[idx[j]]) continue;

                const cv::Rect2f& A = bs[idx[i]];

                const cv::Rect2f& B = bs[idx[j]];

                float ix = std::max(A.x, B.x), iy = std::max(A.y, B.y);

                float ax = std::min(A.x + A.width, B.x + B.width);

                float ay = std::min(A.y + A.height, B.y + B.height);

                float iw = ax - ix, ih = ay - iy;

                if (iw <= 0 || ih <= 0) continue;

                float inter = iw * ih;

                float uni = A.width * A.height + B.width * B.height - inter;

                if (uni > 0 && inter / uni > o.iou) dead[idx[j]] = 1;

            }

        }

        std::vector<Det> r;

        for (int k : keep) {

            Det t;

            t.box = bs[k]; t.conf = sc[k]; t.id = ids[k]; t.kp = kps[k];

            t.name = (t.id >= 0 && t.id < (int)o.names.size())

                         ? o.names[t.id] : ("class" + std::to_string(t.id));

            r.push_back(t);

        }

        return r;

    }

    static void Draw(cv::Mat& img, const std::vector<Det>& ds, const Opt& o) {

        static const cv::Scalar pal[] = {{255,0,0},{0,255,0},{0,0,255},{255,255,0},

                                         {255,0,255},{0,255,255}};

        for (const auto& d : ds) {

            cv::Scalar c = pal[(d.id < 0 ? 0 : d.id) % 6];

            cv::Rect r((int)d.box.x, (int)d.box.y, (int)d.box.width, (int)d.box.height);

            cv::rectangle(img, r, c, 2);

            std::string lb = d.name + " " + cv::format("%.2f", d.conf);

            int bl = 0;

            cv::Size ts = cv::getTextSize(lb, cv::FONT_HERSHEY_SIMPLEX, 0.55, 1, &bl);

            int top = std::max(0, r.y - ts.height - 6);

            cv::rectangle(img, cv::Rect(r.x, top, ts.width + 4, ts.height + 6), c, cv::FILLED);

            cv::putText(img, lb, cv::Point(r.x + 2, top + ts.height + 1),

                        cv::FONT_HERSHEY_SIMPLEX, 0.55, cv::Scalar(255,255,255), 1, cv::LINE_AA);

            for (size_t k = 0; k < d.kp.size(); ++k) {

                cv::Point p((int)d.kp[k].x, (int)d.kp[k].y);

                cv::circle(img, p, 5, cv::Scalar(0,255,0), cv::FILLED);

                cv::putText(img, std::to_string(k + 1), cv::Point(p.x - 7, p.y - 7),

                            cv::FONT_HERSHEY_SIMPLEX, 0.45, cv::Scalar(0,255,0), 1);

            }

        }

        cv::putText(img, "Det: " + std::to_string(ds.size()),

                    cv::Point(10, 28), cv::FONT_HERSHEY_SIMPLEX, 0.7,

                    cv::Scalar(0,255,255), 2);

    }

private:

    cv::dnn::Net net_;

};

// ---------- 全局 UI 状态（滑动条回调需要）----------

static int   g_conf = 25, g_iou = 45, g_exp = 1000, g_gain = 0;

static bool  g_pause = false, g_lb = true;

static float g_emin = 100, g_emax = 100000, g_gmin = 0, g_gmax = 24;

static Cam*  g_cam = nullptr;

static float EUs() { return g_emin + (g_emax - g_emin) * (g_exp / 1000.f); }

static float GDb() { return g_gmin + (g_gmax - g_gmin) * (g_gain / 100.f); }

static void OnExp(int, void*)  { if (g_cam) g_cam->SetExp(EUs()); }

static void OnGain(int, void*) { if (g_cam) g_cam->SetGain(GDb()); }

static cv::Mat Fit(const cv::Mat& s, int w) {

    if (w <= 0 || s.cols <= w) return s;

    cv::Mat o; cv::resize(s, o, cv::Size(), (float)w / s.cols, (float)w / s.cols);

    return o;

}

// ---------- main ----------

int main(int argc, char** argv) {

    std::string model, image, serial, cls = "B1,B2,B3,R1,R2,R3";

    int winW = 1280;

    bool onlyList = false;

    for (int i = 1; i < argc; ++i) {

        std::string a = argv[i];

        auto nx = [&]() -> std::string {

            if (i + 1 >= argc) throw std::runtime_error(a + " 缺参数");

            return argv[++i];

        };

        try {

            if (a == "--help" || a == "-h") {

                std::cout << "用法: ./mvs_yolo [选项]\n"

                             "  --list              列相机\n"

                             "  --model <onnx>      模型\n"

                             "  --image <jpg>       图片测试\n"

                             "  --classes a,b,c     类别名\n"

                             "  --serial <sn>       指定相机\n"

                             "  --width <px>        窗口宽\n";

                return 0;

            } else if (a == "--list")    onlyList = true;

            else if (a == "--model")     model = nx();

            else if (a == "--image")     image = nx();

            else if (a == "--serial")    serial = nx();

            else if (a == "--classes")   cls = nx();

            else if (a == "--width")     winW = std::stoi(nx());

            else { std::cerr << "未知参数 " << a << "\n"; return 2; }

        } catch (const std::exception& e) {

            std::cerr << "[参数错误] " << e.what() << "\n"; return 2;

        }

    }

    Opt o;

    { std::stringstream ss(cls); std::string t;

      while (std::getline(ss, t, ',')) if (!t.empty()) o.names.push_back(t); }

    try {

        // 1) 枚举

        if (onlyList || (model.empty() && image.empty())) {

            std::cout << "=== 相机列表 ===\n";

            auto v = Cam::List();

            if (v.empty()) {

                std::cout << "没找到相机。排查:\n"

                             "  lsusb | grep -i hik\n"

                             "  groups | grep plugdev\n"

                             "  ps aux | grep -i MVS\n";

                return 1;

            }

            for (size_t i = 0; i < v.size(); ++i)

                std::cout << "  [" << i << "] " << v[i].first << " SN=" << v[i].second << "\n";

            if (model.empty() && image.empty()) return 0;

        }

        if (model.empty()) { std::cerr << "需要 --model\n"; return 2; }

        Detector det;

        det.Load(model);

        // 2) 图片测试

        if (!image.empty()) {

            cv::Mat img = cv::imread(image);

            if (img.empty()) { std::cerr << "读不到图 " << image << "\n"; return 1; }

            std::cout << "图片 " << img.cols << "x" << img.rows << "\n";

            auto ds = det.Run(img, o);

            std::cout << "检出 " << ds.size() << " 个\n";

            for (size_t i = 0; i < ds.size(); ++i)

                std::cout << "  " << ds[i].name << " " << ds[i].conf

                          << " kp=" << ds[i].kp.size() << "\n";

            Detector::Draw(img, ds, o);

            cv::imwrite("result.jpg", img);

            std::cout << "已存 result.jpg\n";

            return 0;

        }

        // 3) 相机

        Cam cam;

        g_cam = &cam;

        cam.Open(serial);

        cam.Range(g_emin, g_emax, g_gmin, g_gmax);

        if (g_emax <= g_emin) { g_emin = 100; g_emax = 100000; }

        if (g_gmax <= g_gmin) { g_gmin = 0; g_gmax = 24; }

        std::cout << "曝光范围 " << g_emin << "~" << g_emax

                  << " 增益 " << g_gmin << "~" << g_gmax << "\n";

        float initExp = std::max(g_emin, std::min(g_emax, (g_emin + g_emax) * 0.02f));

        cam.Config(initExp, g_gmin);

        cam.Start();

        // 让滑动条初始位置对应真实值

        g_exp = (int)((initExp - g_emin) / (g_emax - g_emin) * 1000.f);

        g_gain = 0;

        // 4) 界面

        std::string W = "MVS+YOLO  [ESC]quit [s]save [p]pause [b]letterbox";

        cv::namedWindow(W, cv::WINDOW_NORMAL);

        cv::resizeWindow(W, winW, (int)(winW * 0.62));

        cv::createTrackbar("Confidence", W, &g_conf, 100, nullptr);

        cv::createTrackbar("IoU",        W, &g_iou,  100, nullptr);

        cv::createTrackbar("Exposure",   W, &g_exp, 1000, OnExp);

        cv::createTrackbar("Gain",       W, &g_gain, 100, OnGain);

        // 5) 主循环

        std::cout << "\n开始实时推理，ESC 退出\n";

        int nf = 0, to = 0, sv = 0;

        double ms = 0;

        cv::Mat last;

        while (true) {

            o.conf = g_conf / 100.f;

            o.iou  = g_iou / 100.f;

            o.letterbox = g_lb;

            if (g_pause) {

                if (!last.empty()) cv::imshow(W, Fit(last, winW));

                int k = cv::waitKey(30);

                if (k == 27 || k == 'q') break;

                if (k == 'p') g_pause = false;

                continue;

            }

            cv::Mat bgr;

            if (!cam.Grab(bgr, 1000) || bgr.empty()) {

                if (++to >= 5) { std::cerr << "连续5次超时，退出\n"; break; }

                continue;

            }

            to = 0; ++nf;

            auto t0 = std::chrono::steady_clock::now();

            std::vector<Det> ds;

            try { ds = det.Run(bgr, o); }

            catch (const std::exception& e) {

                std::cerr << "[推理错误] " << e.what() << "\n";

                cv::imshow(W, Fit(bgr, winW));

                int k = cv::waitKey(1);

                if (k == 27 || k == 'q') break;

                continue;

            }

            ms = std::chrono::duration<double, std::milli>(

                     std::chrono::steady_clock::now() - t0).count();

            cv::Mat show = bgr.clone();

            Detector::Draw(show, ds, o);

            cv::putText(show, cv::format("FPS %.1f  Infer %.1fms  Frame %d",

                                         ms > 0 ? 1000 / ms : 0, ms, nf),

                        cv::Point(10, 56), cv::FONT_HERSHEY_SIMPLEX, 0.6,

                        cv::Scalar(0,255,255), 1);

            last = show;

            cv::imshow(W, Fit(show, winW));

            int k = cv::waitKey(1);

            if (k == 27 || k == 'q') break;

            if (k == 's') {

                std::string b = "cap_" + std::to_string(sv++);

                cv::imwrite(b + "_raw.jpg", bgr);

                cv::imwrite(b + "_det.jpg", show);

                std::cout << "已存 " << b << "_raw.jpg / _det.jpg\n";

            } else if (k == 'p') { g_pause = true; }

            else if (k == 'b') { g_lb = !g_lb; }

        }

        cam.Stop();

        cv::destroyAllWindows();

        std::cout << "结束，共 " << nf << " 帧\n";

        return 0;

    } catch (const std::exception& e) {

        std::cerr << "\n[错误] " << e.what() << "\n";

        return 1;

    }

}