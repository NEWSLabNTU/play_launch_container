// Copyright 2026 play_launch developers
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include "play_launch_container/control_json.hpp"

#include <cerrno>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cstring>

namespace play_launch_container::json
{

namespace
{
const Value & null_value()
{
  static const Value kNull;
  return kNull;
}
}  // namespace

const Value & Value::operator[](const std::string & key) const
{
  if (type_ != Type::Object) {
    return null_value();
  }
  auto it = object_.find(key);
  return it == object_.end() ? null_value() : it->second;
}

const Value & Value::at(size_t index) const
{
  if (type_ != Type::Array || index >= array_.size()) {
    return null_value();
  }
  return array_[index];
}

size_t Value::size() const
{
  if (type_ == Type::Array) {
    return array_.size();
  }
  if (type_ == Type::Object) {
    return object_.size();
  }
  return 0;
}

std::string Value::as_string(const std::string & fallback) const
{
  return type_ == Type::String ? string_ : fallback;
}

bool Value::as_bool(bool fallback) const { return type_ == Type::Bool ? bool_ : fallback; }

double Value::as_double(double fallback) const
{
  if (type_ != Type::Number) {
    return fallback;
  }
  errno = 0;
  char * end = nullptr;
  const double v = std::strtod(number_.c_str(), &end);
  if (end == number_.c_str()) {
    return fallback;
  }
  return v;
}

int64_t Value::as_int64(int64_t fallback) const
{
  if (type_ != Type::Number) {
    return fallback;
  }
  errno = 0;
  char * end = nullptr;
  const long long v = std::strtoll(number_.c_str(), &end, 10);
  if (end == number_.c_str() || errno == ERANGE) {
    // A JSON number that is not an integer literal (1.5, 1e3) still has a
    // meaningful integer reading; fall through to the double path rather than
    // reporting the fallback.
    const double d = as_double(static_cast<double>(fallback));
    return static_cast<int64_t>(d);
  }
  return static_cast<int64_t>(v);
}

uint64_t Value::as_uint64(uint64_t fallback) const
{
  if (type_ != Type::Number) {
    return fallback;
  }
  errno = 0;
  char * end = nullptr;
  const unsigned long long v = std::strtoull(number_.c_str(), &end, 10);
  if (end == number_.c_str() || errno == ERANGE) {
    return fallback;
  }
  return static_cast<uint64_t>(v);
}

// ── Parser ──────────────────────────────────────────────────────────────

class Parser
{
public:
  Parser(const std::string & text, std::string * error) : s_(text), error_(error) {}

  bool run(Value * out)
  {
    skip_ws();
    if (!parse_value(out)) {
      return false;
    }
    skip_ws();
    if (i_ != s_.size()) {
      return fail("trailing characters after the JSON value");
    }
    return true;
  }

private:
  bool fail(const std::string & msg)
  {
    if (error_ != nullptr && error_->empty()) {
      *error_ = msg + " at offset " + std::to_string(i_);
    }
    return false;
  }

  void skip_ws()
  {
    while (i_ < s_.size()) {
      const char c = s_[i_];
      if (c == ' ' || c == '\t' || c == '\n' || c == '\r') {
        ++i_;
      } else {
        break;
      }
    }
  }

  bool literal(const char * word)
  {
    const size_t n = std::strlen(word);
    if (s_.compare(i_, n, word) != 0) {
      return false;
    }
    i_ += n;
    return true;
  }

  bool parse_value(Value * out)
  {
    if (i_ >= s_.size()) {
      return fail("unexpected end of input");
    }
    switch (s_[i_]) {
      case '{':
        return parse_object(out);
      case '[':
        return parse_array(out);
      case '"': {
        std::string str;
        if (!parse_string(&str)) {
          return false;
        }
        out->type_ = Value::Type::String;
        out->string_ = std::move(str);
        return true;
      }
      case 't':
        if (!literal("true")) {
          return fail("invalid literal");
        }
        out->type_ = Value::Type::Bool;
        out->bool_ = true;
        return true;
      case 'f':
        if (!literal("false")) {
          return fail("invalid literal");
        }
        out->type_ = Value::Type::Bool;
        out->bool_ = false;
        return true;
      case 'n':
        if (!literal("null")) {
          return fail("invalid literal");
        }
        out->type_ = Value::Type::Null;
        return true;
      default:
        return parse_number(out);
    }
  }

  bool parse_object(Value * out)
  {
    ++i_;  // '{'
    out->type_ = Value::Type::Object;
    skip_ws();
    if (i_ < s_.size() && s_[i_] == '}') {
      ++i_;
      return true;
    }
    while (true) {
      skip_ws();
      if (i_ >= s_.size() || s_[i_] != '"') {
        return fail("expected an object key");
      }
      std::string key;
      if (!parse_string(&key)) {
        return false;
      }
      skip_ws();
      if (i_ >= s_.size() || s_[i_] != ':') {
        return fail("expected ':'");
      }
      ++i_;
      skip_ws();
      Value value;
      if (!parse_value(&value)) {
        return false;
      }
      out->object_.emplace(std::move(key), std::move(value));
      skip_ws();
      if (i_ >= s_.size()) {
        return fail("unterminated object");
      }
      if (s_[i_] == ',') {
        ++i_;
        continue;
      }
      if (s_[i_] == '}') {
        ++i_;
        return true;
      }
      return fail("expected ',' or '}'");
    }
  }

  bool parse_array(Value * out)
  {
    ++i_;  // '['
    out->type_ = Value::Type::Array;
    skip_ws();
    if (i_ < s_.size() && s_[i_] == ']') {
      ++i_;
      return true;
    }
    while (true) {
      skip_ws();
      Value value;
      if (!parse_value(&value)) {
        return false;
      }
      out->array_.push_back(std::move(value));
      skip_ws();
      if (i_ >= s_.size()) {
        return fail("unterminated array");
      }
      if (s_[i_] == ',') {
        ++i_;
        continue;
      }
      if (s_[i_] == ']') {
        ++i_;
        return true;
      }
      return fail("expected ',' or ']'");
    }
  }

  /// Append `cp` to `out` as UTF-8.
  static void append_utf8(uint32_t cp, std::string * out)
  {
    if (cp <= 0x7F) {
      out->push_back(static_cast<char>(cp));
    } else if (cp <= 0x7FF) {
      out->push_back(static_cast<char>(0xC0 | (cp >> 6)));
      out->push_back(static_cast<char>(0x80 | (cp & 0x3F)));
    } else if (cp <= 0xFFFF) {
      out->push_back(static_cast<char>(0xE0 | (cp >> 12)));
      out->push_back(static_cast<char>(0x80 | ((cp >> 6) & 0x3F)));
      out->push_back(static_cast<char>(0x80 | (cp & 0x3F)));
    } else {
      out->push_back(static_cast<char>(0xF0 | (cp >> 18)));
      out->push_back(static_cast<char>(0x80 | ((cp >> 12) & 0x3F)));
      out->push_back(static_cast<char>(0x80 | ((cp >> 6) & 0x3F)));
      out->push_back(static_cast<char>(0x80 | (cp & 0x3F)));
    }
  }

  bool parse_hex4(uint32_t * out)
  {
    if (i_ + 4 > s_.size()) {
      return fail("truncated \\u escape");
    }
    uint32_t v = 0;
    for (int k = 0; k < 4; ++k) {
      const char c = s_[i_ + k];
      v <<= 4;
      if (c >= '0' && c <= '9') {
        v |= static_cast<uint32_t>(c - '0');
      } else if (c >= 'a' && c <= 'f') {
        v |= static_cast<uint32_t>(c - 'a' + 10);
      } else if (c >= 'A' && c <= 'F') {
        v |= static_cast<uint32_t>(c - 'A' + 10);
      } else {
        return fail("bad hex digit in \\u escape");
      }
    }
    i_ += 4;
    *out = v;
    return true;
  }

  bool parse_string(std::string * out)
  {
    ++i_;  // opening quote
    out->clear();
    while (true) {
      if (i_ >= s_.size()) {
        return fail("unterminated string");
      }
      const char c = s_[i_++];
      if (c == '"') {
        return true;
      }
      if (c != '\\') {
        out->push_back(c);
        continue;
      }
      if (i_ >= s_.size()) {
        return fail("unterminated escape");
      }
      const char e = s_[i_++];
      switch (e) {
        case '"':
          out->push_back('"');
          break;
        case '\\':
          out->push_back('\\');
          break;
        case '/':
          out->push_back('/');
          break;
        case 'b':
          out->push_back('\b');
          break;
        case 'f':
          out->push_back('\f');
          break;
        case 'n':
          out->push_back('\n');
          break;
        case 'r':
          out->push_back('\r');
          break;
        case 't':
          out->push_back('\t');
          break;
        case 'u': {
          uint32_t cp = 0;
          if (!parse_hex4(&cp)) {
            return false;
          }
          // A surrogate pair arrives as two escapes; join them so the UTF-8
          // that comes out is the character that went in.
          if (cp >= 0xD800 && cp <= 0xDBFF && i_ + 1 < s_.size() && s_[i_] == '\\' &&
              s_[i_ + 1] == 'u') {
            const size_t save = i_;
            i_ += 2;
            uint32_t low = 0;
            if (!parse_hex4(&low)) {
              return false;
            }
            if (low >= 0xDC00 && low <= 0xDFFF) {
              cp = 0x10000 + ((cp - 0xD800) << 10) + (low - 0xDC00);
            } else {
              i_ = save;  // not a pair after all
            }
          }
          append_utf8(cp, out);
          break;
        }
        default:
          return fail("unknown escape");
      }
    }
  }

  bool parse_number(Value * out)
  {
    const size_t start = i_;
    if (i_ < s_.size() && (s_[i_] == '-' || s_[i_] == '+')) {
      ++i_;
    }
    bool any = false;
    while (i_ < s_.size() &&
           ((s_[i_] >= '0' && s_[i_] <= '9') || s_[i_] == '.' || s_[i_] == 'e' || s_[i_] == 'E' ||
            s_[i_] == '-' || s_[i_] == '+')) {
      any = true;
      ++i_;
    }
    if (!any) {
      return fail("expected a value");
    }
    out->type_ = Value::Type::Number;
    out->number_ = s_.substr(start, i_ - start);
    return true;
  }

  const std::string & s_;
  std::string * error_;
  size_t i_ = 0;
};

bool parse(const std::string & text, Value * out, std::string * error)
{
  if (error != nullptr) {
    error->clear();
  }
  *out = Value();
  Parser parser(text, error);
  return parser.run(out);
}

// ── Writer ──────────────────────────────────────────────────────────────

std::string quote(const std::string & s)
{
  std::string out;
  out.reserve(s.size() + 2);
  out.push_back('"');
  for (const unsigned char c : s) {
    switch (c) {
      case '"':
        out += "\\\"";
        break;
      case '\\':
        out += "\\\\";
        break;
      case '\b':
        out += "\\b";
        break;
      case '\f':
        out += "\\f";
        break;
      case '\n':
        out += "\\n";
        break;
      case '\r':
        out += "\\r";
        break;
      case '\t':
        out += "\\t";
        break;
      default:
        if (c < 0x20) {
          char buf[7];
          std::snprintf(buf, sizeof(buf), "\\u%04x", c);
          out += buf;
        } else {
          // Bytes >= 0x80 pass through: the input is already UTF-8 (node
          // names, plugin names, error strings from strerror), and the far
          // end parses UTF-8 JSON.
          out.push_back(static_cast<char>(c));
        }
        break;
    }
  }
  out.push_back('"');
  return out;
}

void ObjectWriter::separate()
{
  if (buf_.empty()) {
    buf_.push_back('{');
  } else {
    buf_.push_back(',');
  }
}

ObjectWriter & ObjectWriter::field(const std::string & key, const std::string & value)
{
  separate();
  buf_ += quote(key);
  buf_.push_back(':');
  buf_ += quote(value);
  return *this;
}

ObjectWriter & ObjectWriter::field(const std::string & key, const char * value)
{
  return field(key, std::string(value == nullptr ? "" : value));
}

ObjectWriter & ObjectWriter::field(const std::string & key, uint64_t value)
{
  separate();
  buf_ += quote(key);
  buf_.push_back(':');
  buf_ += std::to_string(value);
  return *this;
}

ObjectWriter & ObjectWriter::field(const std::string & key, int64_t value)
{
  separate();
  buf_ += quote(key);
  buf_.push_back(':');
  buf_ += std::to_string(value);
  return *this;
}

ObjectWriter & ObjectWriter::field(const std::string & key, int value)
{
  return field(key, static_cast<int64_t>(value));
}

ObjectWriter & ObjectWriter::field(const std::string & key, bool value)
{
  separate();
  buf_ += quote(key);
  buf_.push_back(':');
  buf_ += value ? "true" : "false";
  return *this;
}

std::string ObjectWriter::str() const
{
  if (buf_.empty()) {
    return "{}";
  }
  return buf_ + "}";
}

}  // namespace play_launch_container::json
