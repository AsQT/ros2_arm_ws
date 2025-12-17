#include "fas_protocol.hpp"
#include <cstring>
#include <optional>
#include <cmath>

namespace robot_hardware {

// ====================== CRC16-CCITT ======================
static inline uint16_t crc16_ccitt(const uint8_t* data, size_t len, uint16_t init = 0xFFFF)
{
  uint16_t crc = init;
  for (size_t i = 0; i < len; ++i) {
    crc ^= (uint16_t)data[i] << 8;
    for (int b = 0; b < 8; ++b) {
      if (crc & 0x8000) crc = (uint16_t)((crc << 1) ^ 0x1021);
      else             crc = (uint16_t)(crc << 1);
    }
  }
  return crc;
}

static inline void push_u16_le(std::vector<uint8_t>& out, uint16_t v)
{
  out.push_back((uint8_t)(v & 0xFF));
  out.push_back((uint8_t)((v >> 8) & 0xFF));
}

static inline uint16_t read_u16_le(const uint8_t* p)
{
  return (uint16_t)p[0] | ((uint16_t)p[1] << 8);
}

static inline void push_i32_le(std::vector<uint8_t>& out, int32_t v)
{
  out.push_back((uint8_t)(v & 0xFF));
  out.push_back((uint8_t)((v >> 8) & 0xFF));
  out.push_back((uint8_t)((v >> 16) & 0xFF));
  out.push_back((uint8_t)((v >> 24) & 0xFF));
}

static inline void push_u32_le(std::vector<uint8_t>& out, uint32_t v)
{
  out.push_back((uint8_t)(v & 0xFF));
  out.push_back((uint8_t)((v >> 8) & 0xFF));
  out.push_back((uint8_t)((v >> 16) & 0xFF));
  out.push_back((uint8_t)((v >> 24) & 0xFF));
}

static inline int32_t read_i32_le(const uint8_t* p)
{
  return (int32_t)((uint32_t)p[0]
                 | ((uint32_t)p[1] << 8)
                 | ((uint32_t)p[2] << 16)
                 | ((uint32_t)p[3] << 24));
}

static inline uint32_t read_u32_le(const uint8_t* p)
{
  return (uint32_t)p[0]
       | ((uint32_t)p[1] << 8)
       | ((uint32_t)p[2] << 16)
       | ((uint32_t)p[3] << 24);
}

// ====================== Stuff / Unstuff 0xAA ======================
// Quy ước: byte 0xAA trong data -> 0xAA 0x00
static inline void stuffAA(const std::vector<uint8_t>& in, std::vector<uint8_t>& out)
{
  out.clear();
  out.reserve(in.size() * 2);
  for (uint8_t b : in) {
    if (b == 0xAA) {
      out.push_back(0xAA);
      out.push_back(0x00);
    } else {
      out.push_back(b);
    }
  }
}

static inline bool unstuffAA(const std::vector<uint8_t>& in, std::vector<uint8_t>& out)
{
  out.clear();
  out.reserve(in.size());
  for (size_t i = 0; i < in.size(); ++i) {
    uint8_t b = in[i];
    if (b == 0xAA) {
      if (i + 1 >= in.size()) return false;
      if (in[i + 1] != 0x00) return false; // escape sai
      out.push_back(0xAA);
      ++i; // skip 0x00
    } else {
      out.push_back(b);
    }
  }
  return true;
}

// ====================== Unit conversion ======================
static inline double rad2deg(double rad) { return rad * 180.0 / M_PI; }
static inline double deg2rad(double deg) { return deg * M_PI / 180.0; }

// ====================== PACK FRAME (NEW) ======================
// Format: [AA][CC][CMD][Payload][CRC16][AA][EE]
// CRC16 tính trên (CMD + Payload) raw (chưa stuff), little-endian.
std::vector<uint8_t> packFrame(const Frame& f)
{
  // raw block: [CMD][Payload...][CRC16]
  std::vector<uint8_t> raw;
  raw.reserve(1 + f.payload.size() + 2);

  raw.push_back(static_cast<uint8_t>(f.op));
  raw.insert(raw.end(), f.payload.begin(), f.payload.end());

  const uint16_t crc = crc16_ccitt(raw.data(), raw.size(), 0xFFFF);
  push_u16_le(raw, crc);

  // stuff 0xAA inside raw
  std::vector<uint8_t> stuffed;
  stuffAA(raw, stuffed);

  // frame: header + stuffed + tail
  std::vector<uint8_t> out;
  out.reserve(2 + stuffed.size() + 2);

  out.push_back(0xAA);
  out.push_back(0xCC);
  out.insert(out.end(), stuffed.begin(), stuffed.end());
  out.push_back(0xAA);
  out.push_back(0xEE);

  return out;
}

// ====================== UNPACK FRAME  ======================
// rx có thể chứa 1 phần hoặc nhiều frame -> hàm này tìm 1 frame đầu tiên.
std::optional<Frame> tryUnpackFrame(const std::vector<uint8_t>& rx)
{
  if (rx.size() < 6) return std::nullopt;

  // tìm header [AA CC]
  size_t start = 0;
  while (start + 1 < rx.size()) {
    if (rx[start] == 0xAA && rx[start + 1] == 0xCC) break;
    ++start;
  }
  if (start + 1 >= rx.size()) return std::nullopt;

  // tìm tail [AA EE] sau header
  size_t end = start + 2;
  while (end + 1 < rx.size()) {
    if (rx[end] == 0xAA && rx[end + 1] == 0xEE) break;
    ++end;
  }
  if (end + 1 >= rx.size()) return std::nullopt;

  // bytes giữa header và tail
  if (end < start + 2) return std::nullopt;
  std::vector<uint8_t> mid(rx.begin() + (long)(start + 2), rx.begin() + (long)end);

  // unstuff
  std::vector<uint8_t> raw;
  if (!unstuffAA(mid, raw)) return std::nullopt;

  // raw phải có ít nhất: CMD(1) + CRC(2)
  if (raw.size() < 3) return std::nullopt;

  const uint8_t cmd = raw[0];
  const uint16_t crc_rx = read_u16_le(raw.data() + raw.size() - 2);

  // tính lại CRC trên [CMD + Payload]
  const size_t data_len = raw.size() - 2; // exclude CRC
  const uint16_t crc_cal = crc16_ccitt(raw.data(), data_len, 0xFFFF);
  if (crc_cal != crc_rx) return std::nullopt;

  Frame f;
  f.op = static_cast<OpCode>(cmd);
  f.payload.assign(raw.begin() + 1, raw.begin() + (long)(raw.size() - 2));
  return f;
}

// ====================== PACK CMD POS / VEL (deg*1000) ======================
// ROS input is rad (pos) / rad/s (vel) -> encode to int32 / uint32 in deg*1000.

std::vector<uint8_t> packCmdPosRadToDeg1000I32(const std::vector<double>& pos_rad)
{
  std::vector<uint8_t> payload;
  payload.reserve(pos_rad.size() * 4);

  for (double r : pos_rad) {
    const double deg = rad2deg(r);
    const int32_t raw = (int32_t)std::llround(deg * 1000.0);
    push_i32_le(payload, raw);
  }
  return payload;
}

std::vector<uint8_t> packCmdVelRadToDeg1000U32(const std::vector<double>& vel_rad_s)
{
  std::vector<uint8_t> payload;
  payload.reserve(vel_rad_s.size() * 4);

  for (double rps : vel_rad_s) {
    const double degps = rad2deg(rps);
    double scaled = degps * 1000.0;
    if (scaled < 0) scaled = -scaled; // nếu bạn muốn vel luôn dương (uint32)
    const uint32_t raw = (uint32_t)std::llround(scaled);
    push_u32_le(payload, raw);
  }
  return payload;
}

// ====================== UNPACK TELEMETRY  ======================
// Payload telemetry format (bạn yêu cầu):
// [N*int32 pos_deg1000][N*uint32 vel_deg1000][uint64 flags]
bool unpackTelemetry(const Frame& f, JointSample& js, StatusFlags& st, size_t n)
{
  const size_t need = n * 4 + n * 4 + 8;
  if (f.payload.size() < need) return false;

  js.pos.resize(n);
  js.vel.resize(n);

  const uint8_t* p = f.payload.data();

  // pos int32 deg*1000
  for (size_t i = 0; i < n; ++i) {
    int32_t pos_raw = read_i32_le(p + i * 4);
    double deg = (double)pos_raw / 1000.0;
    js.pos[i] = deg2rad(deg); // publish rad cho /joint_states
  }

  // vel uint32 deg/s*1000
  const uint8_t* pv = p + n * 4;
  for (size_t i = 0; i < n; ++i) {
    uint32_t vel_raw = read_u32_le(pv + i * 4);
    double degps = (double)vel_raw / 1000.0;
    js.vel[i] = deg2rad(degps); // publish rad/s
  }

  // flags uint64 (little-endian)
  uint64_t flags = 0;
  std::memcpy(&flags, p + n * 8, 8);
  st.flags = flags;

  return true;
}

} // namespace robot_hardware
