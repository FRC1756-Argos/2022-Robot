/// \copyright Copyright (c) Argos FRC Team 1756.
///            Open Source Software; you can modify and/or share it under the terms of
///            the license file in the root directory of this project.

#pragma once

#include "argos_lib/general/swerve_utils.h"
#include "wpi/fs.h"

class FileSystemHomingStorage : public argos_lib::swerve::SwerveHomeStorageInterface {
 public:
  explicit FileSystemHomingStorage(const fs::path& swerveHomesPath);
  bool Save(const argos_lib::swerve::SwerveModulePositions& homePosition) override;
  std::optional<argos_lib::swerve::SwerveModulePositions> Load() override;

 private:
  fs::path GetFilePath();
  const fs::path m_swerveHomesPath;
};
