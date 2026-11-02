#pragma once
// Tiny bounded JSON object writer. No Arduino dependency.
#include <stdio.h>
#include <stdint.h>
#include <stddef.h>
#include <string.h>

namespace hrv {

class JsonOut {
 public:
  JsonOut(char* buf, size_t cap) : b_(buf), cap_(cap), n_(0), first_(true), over_(false) {
    if (cap_) b_[0] = 0;
  }
  void begin() { n_ = 0; first_ = true; over_ = false; raw("{"); }
  void end() { raw("}"); }

  void num(const char* key, double v, int decimals, bool valid) {
    if (!valid) { nul(key); return; }
    key_(key);
    char t[40];
    snprintf(t, sizeof t, "%.*f", decimals, v);
    raw(t);
  }
  void integer(const char* key, long v, bool valid) {
    if (!valid) { nul(key); return; }
    key_(key);
    char t[24];
    snprintf(t, sizeof t, "%ld", v);
    raw(t);
  }
  void boolean(const char* key, bool v, bool valid) {
    if (!valid) { nul(key); return; }
    key_(key);
    raw(v ? "true" : "false");
  }
  // v == nullptr writes null. Quotes, backslashes and control chars become '_'.
  void str(const char* key, const char* v) {
    if (!v) { nul(key); return; }
    key_(key);
    raw("\"");
    for (const char* p = v; *p; ++p) {
      char c = *p;
      if (c == '"' || c == '\\' || (unsigned char)c < 0x20) c = '_';
      char t[2] = { c, 0 };
      raw(t);
    }
    raw("\"");
  }

  const char* c_str() const { return b_; }
  bool ok() const { return !over_; }

 private:
  void nul(const char* key) { key_(key); raw("null"); }
  void key_(const char* key) {
    if (!first_) raw(",");
    first_ = false;
    raw("\""); raw(key); raw("\":");
  }
  void raw(const char* s) {
    size_t l = strlen(s);
    if (n_ + l + 1 > cap_) { over_ = true; return; }
    memcpy(b_ + n_, s, l);
    n_ += l;
    b_[n_] = 0;
  }

  char* b_;
  size_t cap_, n_;
  bool first_, over_;
};

}  // namespace hrv
