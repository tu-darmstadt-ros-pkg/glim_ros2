#pragma once

#include <glim/util/callback_slot.hpp>

namespace glim {

/**
 * @brief Callbacks emitted by the GlimROS node itself (as opposed to the mapping modules)
 */
struct GlimROSCallbacks {
  /**
   * @brief Called when the mapping pipeline has been torn down and is about to be rebuilt
   *        (see GlimROS::reset_pipeline).
   * @note  Extension modules stay loaded across a reset because their callbacks live in global
   *        slots. They must use this callback to drop any cached trajectory or map state, which
   *        belongs to the destroyed pipeline.
   * @note  Called from the thread that requested the reset while no mapping module is running.
   */
  static CallbackSlot<void()> on_reset;
};

}  // namespace glim
