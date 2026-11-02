#pragma once
// Relay safety logic. Plain C++ with no Arduino dependency so it can be unit
// tested on a PC. All times are millis()-style uint32_t and wrap-safe.
#include <stdint.h>

namespace hrv {

constexpr int kMaxChannels = 8;
constexpr int kSwitchChannels = 3;   // channels 0..2 = S1..S3 (interlocked)

struct ChannelLimits {
  uint32_t defaultOnMs;
  uint32_t maxOnMs;
};

enum class Result : uint8_t { Ok = 0, BadChannel, BootHold, Blocked, Interlock, TooSoon };

inline const char* resultName(Result r) {
  switch (r) {
    case Result::Ok: return "ok";
    case Result::BadChannel: return "rejected: bad channel";
    case Result::BootHold: return "rejected: boot hold";
    case Result::Blocked: return "rejected: input function not allowed or unknown";
    case Result::Interlock: return "rejected: another switch is closed";
    case Result::TooSoon: return "rejected: too soon after last change";
  }
  return "?";
}

// Elapsed time since t, never negative: if `now` is older than t (a caller reused a
// stale clock reading) it counts as 0 instead of wrapping to ~49 days.
inline uint32_t since(uint32_t now, uint32_t t) {
  uint32_t d = now - t;
  return ((int32_t)d < 0) ? 0 : d;
}

class RelayGuard {
 public:
  RelayGuard(const ChannelLimits* limits, int channels, int maxClosedSwitches,
             uint32_t bootHoldMs, uint32_t minOffMs, uint32_t minGapMs)
      : limits_(limits),
        n_(channels > kMaxChannels ? kMaxChannels : channels),
        maxClosed_(clampClosed(maxClosedSwitches)),
        bootHoldMs_(bootHoldMs), minOffMs_(minOffMs), minGapMs_(minGapMs),
        bootAt_(0), lastChange_(0) {
    for (int i = 0; i < kMaxChannels; i++) {
      on_[i] = false; blocked_[i] = false; onSince_[i] = 0; onFor_[i] = 0; lastOff_[i] = 0;
    }
  }

  void begin(uint32_t now) {
    bootAt_ = now;
    lastChange_ = now - minGapMs_;
    for (int i = 0; i < kMaxChannels; i++) {
      on_[i] = false; onSince_[i] = 0; onFor_[i] = 0; lastOff_[i] = now - minOffMs_;
    }
  }

  // Ask for a channel to close (on=true) for durationMs (0 = channel default,
  // always clamped to the channel maximum) or to open. Opening is always allowed.
  Result request(int ch, bool on, uint32_t durationMs, uint32_t now) {
    if (ch < 0 || ch >= n_) return Result::BadChannel;
    if (!on) {
      if (on_[ch]) open(ch, now);
      return Result::Ok;
    }
    if (since(now, bootAt_) < bootHoldMs_) return Result::BootHold;
    if (blocked_[ch]) return Result::Blocked;
    uint32_t dur = durationMs ? durationMs : limits_[ch].defaultOnMs;
    if (dur > limits_[ch].maxOnMs) dur = limits_[ch].maxOnMs;
    if (on_[ch]) {                       // already closed: refresh the timer
      onSince_[ch] = now;
      onFor_[ch] = dur;
      return Result::Ok;
    }
    if (since(now, lastOff_[ch]) < minOffMs_) return Result::TooSoon;
    if (since(now, lastChange_) < minGapMs_) return Result::TooSoon;
    if (ch < kSwitchChannels && closedSwitches() >= maxClosed_) return Result::Interlock;
    on_[ch] = true;
    onSince_[ch] = now;
    onFor_[ch] = dur;
    lastChange_ = now;
    return Result::Ok;
  }

  // Open channels whose time is up, or which have become blocked.
  // Returns a bit mask of channels that were opened.
  uint8_t tick(uint32_t now) {
    uint8_t mask = 0;
    for (int i = 0; i < n_; i++) {
      if (on_[i] && (blocked_[i] || since(now, onSince_[i]) >= onFor_[i])) {
        open(i, now);
        mask |= (uint8_t)(1u << i);
      }
    }
    return mask;
  }

  uint8_t allOff(uint32_t now) {
    uint8_t mask = 0;
    for (int i = 0; i < n_; i++) {
      if (on_[i]) { open(i, now); mask |= (uint8_t)(1u << i); }
    }
    return mask;
  }

  void setBlocked(int ch, bool b) { if (ch >= 0 && ch < n_) blocked_[ch] = b; }
  bool isBlocked(int ch) const { return ch >= 0 && ch < n_ && blocked_[ch]; }
  bool isOn(int ch) const { return ch >= 0 && ch < n_ && on_[ch]; }
  int channels() const { return n_; }
  int maxClosed() const { return maxClosed_; }

  uint32_t remainingMs(int ch, uint32_t now) const {
    if (!isOn(ch)) return 0;
    uint32_t el = since(now, onSince_[ch]);
    return el >= onFor_[ch] ? 0 : onFor_[ch] - el;
  }

  uint32_t elapsedOnMs(int ch, uint32_t now) const {
    return isOn(ch) ? since(now, onSince_[ch]) : 0;
  }

  int closedSwitches() const {
    int c = 0;
    for (int i = 0; i < kSwitchChannels && i < n_; i++) if (on_[i]) c++;
    return c;
  }

 private:
  static int clampClosed(int v) {
    if (v < 1) return 1;
    if (v > kSwitchChannels - 1) return kSwitchChannels - 1;   // never all three
    return v;
  }
  void open(int ch, uint32_t now) {
    on_[ch] = false;
    lastOff_[ch] = now;
    lastChange_ = now;
  }

  const ChannelLimits* limits_;
  int n_;
  int maxClosed_;
  uint32_t bootHoldMs_, minOffMs_, minGapMs_;
  uint32_t bootAt_, lastChange_;
  bool on_[kMaxChannels];
  bool blocked_[kMaxChannels];
  uint32_t onSince_[kMaxChannels];
  uint32_t onFor_[kMaxChannels];
  uint32_t lastOff_[kMaxChannels];
};

}  // namespace hrv
