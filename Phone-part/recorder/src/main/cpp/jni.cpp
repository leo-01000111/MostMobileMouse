// JNI bridge: eu.leongorecki.deskmouse.recorder.NativeEngine <-> deskmouse::Engine.
#include <jni.h>

#include <cmath>
#include <limits>
#include <memory>
#include <mutex>
#include <string>

#include "deskmouse/engine.hpp"

using namespace deskmouse;

namespace {
struct Handle {
    std::unique_ptr<Engine> eng;
    int w, h;
    int last_inliers = 0;
};
Handle* H(jlong p) { return reinterpret_cast<Handle*>(p); }
}  // namespace

extern "C" {

JNIEXPORT jlong JNICALL Java_eu_leongorecki_deskmouse_recorder_NativeEngine_nCreate(
    JNIEnv* env, jclass, jdoubleArray jK, jdoubleArray jdist, jint w, jint h, jobjectArray names, jdoubleArray values,
    jdouble gauge_ratio) {
    cv::Matx33d K;
    env->GetDoubleArrayRegion(jK, 0, 9, K.val);
    std::array<double, 5> dist{};
    env->GetDoubleArrayRegion(jdist, 0, 5, dist.data());
    EngineConfig ec;
    FrontEndConfig fc;
    const jsize n = env->GetArrayLength(names);
    std::vector<double> vals(n);
    env->GetDoubleArrayRegion(values, 0, n, vals.data());
    for (jsize i = 0; i < n; ++i) {
        auto js = static_cast<jstring>(env->GetObjectArrayElement(names, i));
        const char* s = env->GetStringUTFChars(js, nullptr);
        set_param(ec, fc, s, vals[i]);  // unknown names are ignored
        env->ReleaseStringUTFChars(js, s);
        env->DeleteLocalRef(js);
    }
    auto* hd = new Handle{std::make_unique<Engine>(K, dist, cv::Size(w, h), ec, fc, gauge_ratio), w, h};
    return reinterpret_cast<jlong>(hd);
}

JNIEXPORT void JNICALL Java_eu_leongorecki_deskmouse_recorder_NativeEngine_nDestroy(JNIEnv*, jclass, jlong p) {
    delete H(p);
}

// Batch of IMU samples in arrival order: kinds 0 = gyro, 1 = accel; xyz interleaved. poll() after each, like the PC.
JNIEXPORT void JNICALL Java_eu_leongorecki_deskmouse_recorder_NativeEngine_nImu(
    JNIEnv* env, jclass, jlong p, jlongArray jt, jbyteArray jk, jfloatArray jxyz, jint n) {
    std::vector<jlong> t(n);
    std::vector<jbyte> k(n);
    std::vector<jfloat> v(3 * size_t(n));
    env->GetLongArrayRegion(jt, 0, n, t.data());
    env->GetByteArrayRegion(jk, 0, n, k.data());
    env->GetFloatArrayRegion(jxyz, 0, 3 * n, v.data());
    auto& e = *H(p)->eng;
    for (jint i = 0; i < n; ++i) {
        e.on_imu(t[i], k[i], v[3 * i], v[3 * i + 1], v[3 * i + 2]);
        e.poll(t[i]);
    }
}

JNIEXPORT void JNICALL Java_eu_leongorecki_deskmouse_recorder_NativeEngine_nFrame(
    JNIEnv* env, jclass, jlong p, jbyteArray jy, jlong t, jlong exposure, jlong skew) {
    auto* hd = H(p);
    jbyte* y = env->GetByteArrayElements(jy, nullptr);
    cv::Mat img(hd->h, hd->w, CV_8UC1, y);
    hd->eng->on_frame(img, t, exposure, skew);
    env->ReleaseByteArrayElements(jy, y, JNI_ABORT);
    hd->last_inliers = hd->eng->last_result.n_inliers;
}

// Events packed as [kind, dx, dy, wheel] × n; kind = deskmouse::EventKind.
JNIEXPORT jintArray JNICALL Java_eu_leongorecki_deskmouse_recorder_NativeEngine_nDrain(JNIEnv* env, jclass, jlong p) {
    const auto ev = H(p)->eng->drain();
    std::vector<jint> out;
    out.reserve(ev.size() * 4);
    for (const auto& e : ev) { out.push_back(int(e.kind)); out.push_back(e.dx); out.push_back(e.dy); out.push_back(e.wheel); }
    jintArray a = env->NewIntArray(jsize(out.size()));
    env->SetIntArrayRegion(a, 0, jsize(out.size()), out.data());
    return a;
}

// [still, lifted, scrolling, scale (NaN = none), frames, taps, scale_updates, lifts, inliers, gauge_ratio (NaN = none)]
JNIEXPORT jdoubleArray JNICALL Java_eu_leongorecki_deskmouse_recorder_NativeEngine_nState(JNIEnv* env, jclass, jlong p) {
    auto* hd = H(p);
    auto& e = *hd->eng;
    const double nan = std::numeric_limits<double>::quiet_NaN();
    const double s[10] = {double(e.still()), double(e.lifted()), double(e.scrolling()), e.scale().value_or(nan),
                          double(e.stats["frames"]), double(e.stats["taps"]), double(e.stats["scale_updates"]),
                          double(e.stats["lifts"]), double(hd->last_inliers), e.gauge_ratio().value_or(nan)};
    jdoubleArray a = env->NewDoubleArray(10);
    env->SetDoubleArrayRegion(a, 0, 10, s);
    return a;
}

}  // extern "C"
