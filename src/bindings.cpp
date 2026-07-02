// Emscripten Embind interface — the JS <-> WASM bridge.
// This is the from-scratch analogue of Niantic's narrow c8EmAsm_* bridge
// (engineInit / stageFrame / processStagedFrame / query).
#include <emscripten/bind.h>
#include <emscripten/val.h>

#include "slam_engine.h"

using namespace emscripten;
using webslam::SlamEngine;

namespace {

// Return the flat keypoint buffer as a Float32Array view into WASM memory.
// Valid until the next processFrame() call; the page reads it immediately.
val keypointsView(const SlamEngine& e) {
  const auto& v = e.keypointsFlat();
  return val(typed_memory_view(v.size(), v.data()));
}

// Return the RGBA input buffer as a Uint8Array view into WASM memory. The page
// writes each camera frame straight into this (no HEAPU8 poking needed, and it
// stays valid across memory growth because we rebuild the view each frame).
val inputView(SlamEngine& e) {
  return val(typed_memory_view(e.inputSize(), e.inputData()));
}

val depthView(SlamEngine& e) {
  return val(typed_memory_view(e.depthSize(), e.depthData()));
}

val matchLinesView(const SlamEngine& e) {
  const auto& v = e.matchLinesFlat();
  return val(typed_memory_view(v.size(), v.data()));
}

val translationDirView(const SlamEngine& e) {
  const auto& v = e.translationDir();
  return val(typed_memory_view(v.size(), v.data()));
}

val accumulatedRotationView(const SlamEngine& e) {
  const auto& v = e.accumulatedRotation();
  return val(typed_memory_view(v.size(), v.data()));
}

val mapPointsView(const SlamEngine& e) {
  const auto& v = e.mapPointsFlat();
  return val(typed_memory_view(v.size(), v.data()));
}

val trajectoryView(const SlamEngine& e) {
  const auto& v = e.trajectoryFlat();
  return val(typed_memory_view(v.size(), v.data()));
}

val poseRView(const SlamEngine& e) {
  const auto& v = e.poseR();
  return val(typed_memory_view(v.size(), v.data()));
}

val poseTView(const SlamEngine& e) {
  const auto& v = e.poseT();
  return val(typed_memory_view(v.size(), v.data()));
}

val stageTimesView(const SlamEngine& e) {
  const auto& v = e.stageTimesFlat();
  return val(typed_memory_view(v.size(), v.data()));
}

}  // namespace

EMSCRIPTEN_BINDINGS(webslam_module) {
  class_<SlamEngine>("SlamEngine")
      .constructor<int, int, int>()
      .function("inputBufferPtr", &SlamEngine::inputBufferPtr)
      .function("inputView", &inputView)
      .function("setIntrinsics", &SlamEngine::setIntrinsics)
      .function("processFrame", &SlamEngine::processFrame)
      .function("stageTimes", &stageTimesView)
      .function("keypoints", &keypointsView)
      .function("trackingOk", &SlamEngine::trackingOk)
      .function("trackingInliers", &SlamEngine::trackingInliers)
      .function("relativeRotationDeg", &SlamEngine::relativeRotationDeg)
      .function("matchLines", &matchLinesView)
      .function("translationDir", &translationDirView)
      .function("accumulatedRotation", &accumulatedRotationView)
      .function("enableMapping", &SlamEngine::enableMapping)
      .function("depthView", &depthView)
      .function("densifyFromDepth", &SlamEngine::densifyFromDepth)
      .function("setMotionHint", &SlamEngine::setMotionHint)
      .function("setGyroDelta", &SlamEngine::setGyroDelta)
      .function("mapState", &SlamEngine::mapState)
      .function("mapTracked", &SlamEngine::mapTracked)
      .function("mapNumPoints", &SlamEngine::mapNumPoints)
      .function("mapNumKeyframes", &SlamEngine::mapNumKeyframes)
      .function("mapSpread", &SlamEngine::mapSpread)
      .function("coverageYawDeg", &SlamEngine::coverageYawDeg)
      .function("coveragePitchDeg", &SlamEngine::coveragePitchDeg)
      .function("mapPoints", &mapPointsView)
      .function("trajectory", &trajectoryView)
      .function("poseR", &poseRView)
      .function("poseT", &poseTView)
      .function("addScaleSample", &SlamEngine::addScaleSample)
      .function("metricScale", &SlamEngine::metricScale)
      .function("scaleConfidence", &SlamEngine::scaleConfidence)
      .function("scaleValid", &SlamEngine::scaleValid)
      .function("scaleSamples", &SlamEngine::scaleSamples)
      .function("scaleAxis", &SlamEngine::scaleAxis)
      .function("resetScale", &SlamEngine::resetScale)
      .function("addImuSample", &SlamEngine::addImuSample)
      .function("viOk", &SlamEngine::viOk)
      .function("viScale", &SlamEngine::viScale)
      .function("viGravityX", &SlamEngine::viGravityX)
      .function("viGravityY", &SlamEngine::viGravityY)
      .function("viGravityZ", &SlamEngine::viGravityZ)
      .function("viGravityMag", &SlamEngine::viGravityMag)
      .function("viConfidence", &SlamEngine::viConfidence)
      .function("viKeyframes", &SlamEngine::viKeyframes)
      .function("resetVi", &SlamEngine::resetVi);
}
