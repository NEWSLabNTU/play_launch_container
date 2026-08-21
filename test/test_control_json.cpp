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

#include <gtest/gtest.h>

#include <string>

#include "play_launch_container/control_json.hpp"

using play_launch_container::json::ObjectWriter;
using play_launch_container::json::Value;
using play_launch_container::json::parse;

namespace
{
Value parse_ok(const std::string & text)
{
  Value v;
  std::string error;
  EXPECT_TRUE(parse(text, &v, &error)) << error << " in: " << text;
  return v;
}
}  // namespace

/// A load frame as serde_json actually emits it: tagged enum, arrays, and
/// only the value field the parameter's type calls for.
TEST(ControlJson, ParsesALoadFrame)
{
  const std::string frame =
    R"({"t":"load","seq":7,"package":"pkg","plugin":"pkg::Plugin","node_name":"n",)"
    R"("node_namespace":"/ns","remap_rules":["a:=b","c:=d"],)"
    R"("parameters":[{"name":"rate","type":3,"double_value":10.5}],)"
    R"("extra_arguments":[{"name":"use_intra_process_comms","type":1,"bool_value":true}],)"
    R"("log_dir":"/var/log/x"})";

  const Value v = parse_ok(frame);
  ASSERT_TRUE(v.is_object());
  EXPECT_EQ(v["t"].as_string(), "load");
  EXPECT_EQ(v["seq"].as_uint64(), 7u);
  EXPECT_EQ(v["package"].as_string(), "pkg");
  EXPECT_EQ(v["node_namespace"].as_string(), "/ns");
  ASSERT_EQ(v["remap_rules"].size(), 2u);
  EXPECT_EQ(v["remap_rules"].at(1).as_string(), "c:=d");
  ASSERT_EQ(v["parameters"].size(), 1u);
  EXPECT_EQ(v["parameters"].at(0)["type"].as_uint64(), 3u);
  EXPECT_DOUBLE_EQ(v["parameters"].at(0)["double_value"].as_double(), 10.5);
  EXPECT_TRUE(v["extra_arguments"].at(0)["bool_value"].as_bool());
  EXPECT_EQ(v["log_dir"].as_string(), "/var/log/x");
}

/// Absent keys must read as absent, not as an error and not as a wrong value:
/// the supervisor omits every optional field it has nothing to say about.
TEST(ControlJson, MissingKeysAreNullAndFallBack)
{
  const Value v = parse_ok(R"({"t":"load","seq":1})");
  EXPECT_TRUE(v["parameters"].is_null());
  EXPECT_EQ(v["parameters"].size(), 0u);
  EXPECT_EQ(v["log_dir"].as_string("fallback"), "fallback");
  EXPECT_EQ(v["nope"]["deeper"].as_uint64(42u), 42u);
}

/// A uint64 beyond double's exact range must survive: unique_ids are u64 on
/// the wire, and reading one through a double would silently round it.
TEST(ControlJson, LargeIntegersAreExact)
{
  const Value v = parse_ok(R"({"unique_id":18446744073709551615})");
  EXPECT_EQ(v["unique_id"].as_uint64(), 18446744073709551615ull);

  const Value w = parse_ok(R"({"n":9007199254740993})");
  EXPECT_EQ(w["n"].as_uint64(), 9007199254740993ull);
}

TEST(ControlJson, ParsesEscapesAndUnicode)
{
  const Value v = parse_ok(R"({"error":"line\none\ttab \"quoted\" \\ é 😀"})");
  EXPECT_EQ(v["error"].as_string(), "line\none\ttab \"quoted\" \\ \xc3\xa9 \xf0\x9f\x98\x80");
}

TEST(ControlJson, RejectsMalformedInput)
{
  for (const char * bad : {"", "{", R"({"a":})", R"({"a" 1})", R"({"a":1} trailing)", "[1,2",
                           R"({"a":"unterminated})"}) {
    Value v;
    std::string error;
    EXPECT_FALSE(parse(bad, &v, &error)) << "accepted: " << bad;
    EXPECT_FALSE(error.empty());
  }
}

/// What the container writes has to be readable by serde_json on the other
/// side, which means correct escaping — an error message carrying a quote or a
/// newline is the normal case, not an edge case (strerror, ctor exceptions).
TEST(ControlJson, WritesEscapedObjects)
{
  ObjectWriter w;
  w.field("t", "load_failed")
    .field("unique_id", static_cast<uint64_t>(12))
    .field("error", "he said \"no\"\nand\tleft\\")
    .field("pid", 4321)
    .field("ok", false);

  const std::string out = w.str();
  EXPECT_EQ(
    out,
    R"({"t":"load_failed","unique_id":12,"error":"he said \"no\"\nand\tleft\\","pid":4321,)"
    R"("ok":false})");

  // Round trip: what we wrote parses back to what we meant.
  const Value back = parse_ok(out);
  EXPECT_EQ(back["error"].as_string(), "he said \"no\"\nand\tleft\\");
  EXPECT_EQ(back["pid"].as_int64(), 4321);
  EXPECT_FALSE(back["ok"].as_bool(true));
}

TEST(ControlJson, EmptyObjectWritesAsEmpty)
{
  EXPECT_EQ(ObjectWriter().str(), "{}");
}

/// Control characters must be \u-escaped rather than emitted raw: a raw 0x01
/// in a string is invalid JSON and would break the whole frame, not just the
/// field.
TEST(ControlJson, EscapesControlCharacters)
{
  ObjectWriter w;
  w.field("e", std::string("a\x01\x1f", 3));
  EXPECT_EQ(w.str(), R"({"e":"a\u0001\u001f"})");
  EXPECT_EQ(parse_ok(w.str())["e"].as_string(), std::string("a\x01\x1f", 3));
}
