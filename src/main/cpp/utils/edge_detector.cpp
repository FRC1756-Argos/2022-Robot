/// \copyright Copyright (c) Argos FRC Team 1756.
///            Open Source Software; you can modify and/or share it under the terms of
///            the license file in the root directory of this project.

#include "utils/edge_detector.h"

EdgeDetector::EdgeDetector(EdgeDetector::EdgeDetectSettings _settings, bool initialValue) {
  m_settings = _settings;
  m_previousValue = initialValue;
}

bool EdgeDetector::operator()(bool curVal) {
  switch (m_settings) {
    case EdgeDetectSettings::DETECT_BOTH:
      return (Calculate(curVal) == edgeStatus::RISING || Calculate(curVal) == edgeStatus::FALLING) ? true : false;
      break;
    case EdgeDetectSettings::DETECT_RISING:
      return (Calculate(curVal) == edgeStatus::RISING) ? true : false;
      break;
    case EdgeDetectSettings::DETECT_FALLING:
      return (Calculate(curVal) == edgeStatus::FALLING) ? true : false;
      break;
  }
  return false;
}

EdgeDetector::edgeStatus EdgeDetector::Calculate(bool curVal) {
  switch (m_settings) {
    case EdgeDetectSettings::DETECT_BOTH:
      if (DetectFalling(curVal) == edgeStatus::FALLING) {
        return edgeStatus::FALLING;
      } else if (DetectRising(curVal) == edgeStatus::RISING) {
        return edgeStatus::RISING;
      } else {
        return edgeStatus::NONE;
      }
      break;
    case EdgeDetectSettings::DETECT_FALLING:
      return DetectFalling(curVal);
      break;
    case EdgeDetectSettings::DETECT_RISING:
      return DetectRising(curVal);
      break;
  }
  m_previousValue = curVal;
  return edgeStatus::NONE;
}

std::string EdgeDetector::ToString(edgeStatus status) {
  switch (status) {
    case edgeStatus::RISING:
      return "Rising";
      break;
    case edgeStatus::FALLING:
      return "Falling";
      break;
    case edgeStatus::NONE:
      return "None";
      break;
    case edgeStatus::ERROR:
      return "Error";
      break;

    default:
      return "DEFAULT";
      break;
  }
}

EdgeDetector::edgeStatus EdgeDetector::DetectFalling(bool currentValue) {
  if (m_previousValue && !currentValue) {
    return edgeStatus::FALLING;
  } else {
    return edgeStatus::NONE;
  }
}

EdgeDetector::edgeStatus EdgeDetector::DetectRising(bool currentValue) {
  if (!m_previousValue && currentValue) {
    return edgeStatus::RISING;
  } else {
    return edgeStatus::NONE;
  }
}
