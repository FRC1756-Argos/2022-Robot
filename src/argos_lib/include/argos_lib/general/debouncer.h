/// \copyright Copyright (c) Argos FRC Team 1756.
///            Open Source Software; you can modify and/or share it under the terms of
///            the license file in the root directory of this project.

#pragma once

#include <chrono>

#include "argos_lib/general/debounce_settings.h"
#include "units/time.h"

namespace argos_lib {

  class Debouncer {
   public:
    /**
     * @brief Construct a new Debouncer object with asymmetric times
     *
     * @param debounceSettings Configuration settings for activate and clear times
     */
    explicit Debouncer(const argos_lib::DebounceSettings debounceSettings);

    /**
     * @brief Construct a new Debouncer object with symmetric times
     *
     * @param symmetricDebounceTime Symmetric activate and clear time
     */
    explicit Debouncer(const units::millisecond_t symmetricDebounceTime);

    /**
     * @brief Update debouncer with new input and retreive latest debounced status
     *
     * @param newVal Latest raw value
     * @return Value after applying debounce
     */
    [[nodiscard]] bool operator()(bool newVal);

   private:
    argos_lib::DebounceSettings m_debounceSettings;  ///< Configuration for asymmetric debouncer
    bool m_rawStatus;                                ///< Last raw value
    bool m_debouncedStatus;                          ///< Current value after applying debounce
    std::chrono::time_point<std::chrono::steady_clock>
        m_debounceTransitionTime;  ///< Time when latest transition detected
  };

}  // namespace argos_lib
