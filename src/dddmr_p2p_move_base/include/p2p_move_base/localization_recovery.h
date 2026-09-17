#ifndef P2P_LOCALIZATION_RECOVERY_H
#define P2P_LOCALIZATION_RECOVERY_H
#include <cstdint>
namespace p2p_move_base {
// Access under the same mutex as localization diagnostics. Times are nanoseconds.
class LocalizationRecovery {
 public:
  void observe(bool valid, int64_t now, int64_t stamp, int64_t previous_expiry) {
    if (!valid) {
      since_ = 0;
    } else if (since_ == 0 || now >= previous_expiry || stamp < last_stamp_) {
      since_ = now;
      first_stamp_ = stamp;
    }
    last_stamp_ = stamp;
  }
  bool ready(int64_t now, int64_t expiry, double stable_seconds) const {
    return now < expiry && since_ > 0 && last_stamp_ > first_stamp_ &&
      now - since_ >= stable_seconds * 1e9;
  }
 private:
  int64_t since_ = 0, first_stamp_ = 0, last_stamp_ = 0;
};
}
#endif
