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

#ifndef PLAY_LAUNCH_CONTAINER__CONTROL_JSON_HPP_
#define PLAY_LAUNCH_CONTAINER__CONTROL_JSON_HPP_

#include <cstdint>
#include <map>
#include <string>
#include <vector>

/// Phase 64: the smallest JSON reader/writer that can carry the container
/// control protocol.
///
/// It is here rather than as a dependency on purpose. `play_launch_container`
/// declares six ament packages and nothing else; adding nlohmann-json or
/// yaml-cpp would put a new rosdep key in front of every source build and a
/// new apt package in the release image, to parse messages whose grammar is
/// six field types wide. The protocol is defined once, in
/// `src/play_launch/src/ipc/container_protocol.rs`, and both ends are ours.
namespace play_launch_container::json
{

/// A parsed JSON value. Copyable; small enough that the control protocol
/// never notices.
class Value
{
public:
  enum class Type { Null, Bool, Number, String, Array, Object };

  Value() = default;

  Type type() const { return type_; }
  bool is_null() const { return type_ == Type::Null; }
  bool is_object() const { return type_ == Type::Object; }
  bool is_array() const { return type_ == Type::Array; }
  bool is_string() const { return type_ == Type::String; }
  bool is_number() const { return type_ == Type::Number; }
  bool is_bool() const { return type_ == Type::Bool; }

  /// Object lookup. A missing key yields a Null value rather than throwing,
  /// so a reader can ask for optional fields without branching first.
  const Value & operator[](const std::string & key) const;

  /// Array access. Out of range yields Null.
  const Value & at(size_t index) const;
  size_t size() const;

  /// Typed reads. Each returns the fallback when this value is of another
  /// type — absent and wrong-typed are the same thing to a reader that has
  /// already agreed on the schema.
  std::string as_string(const std::string & fallback = "") const;
  bool as_bool(bool fallback = false) const;
  double as_double(double fallback = 0.0) const;
  int64_t as_int64(int64_t fallback = 0) const;
  uint64_t as_uint64(uint64_t fallback = 0) const;

  const std::vector<Value> & items() const { return array_; }

private:
  friend class Parser;

  Type type_ = Type::Null;
  bool bool_ = false;
  /// Kept as text so a uint64 that a double cannot hold exactly still reads
  /// back exactly.
  std::string number_;
  std::string string_;
  std::vector<Value> array_;
  std::map<std::string, Value> object_;
};

/// Parse one JSON document. Returns false and fills `error` on failure; the
/// caller decides what a bad frame means (the control channel skips it).
bool parse(const std::string & text, Value * out, std::string * error);

/// Quote and escape a string for JSON output.
std::string quote(const std::string & s);

/// Build a JSON object one field at a time. Deliberately not a tree: every
/// message this container sends is a flat object of scalars.
class ObjectWriter
{
public:
  ObjectWriter & field(const std::string & key, const std::string & value);
  ObjectWriter & field(const std::string & key, const char * value);
  ObjectWriter & field(const std::string & key, uint64_t value);
  ObjectWriter & field(const std::string & key, int64_t value);
  ObjectWriter & field(const std::string & key, int value);
  ObjectWriter & field(const std::string & key, bool value);

  std::string str() const;

private:
  void separate();

  std::string buf_;
};

}  // namespace play_launch_container::json

#endif  // PLAY_LAUNCH_CONTAINER__CONTROL_JSON_HPP_
