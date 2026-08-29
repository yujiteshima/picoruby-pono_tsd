#!/bin/sh
# Host-side test runner. Needs a POSIX picoruby build that includes
# picoruby-pono_tsd (parser test) and picoruby-dsp (pipeline test).
#
#   PICORUBY=../picoruby/bin/picoruby hosttest/run.sh
set -e

DIR=$(cd "$(dirname "$0")" && pwd)
PICORUBY=${PICORUBY:-$DIR/../../picoruby/bin/picoruby}

run() {
  echo "--- $1"
  out=$("$PICORUBY" "$2")
  echo "$out"
  echo "$out" | grep -q "ALL OK" || exit 1
}

run "parser_check" "$DIR/parser_check.rb"

TMP=$(mktemp "${TMPDIR:-/tmp}/pono_tsd_pipeline.XXXXXX")
cat "$DIR/fixture_fan.rb" "$DIR/pipeline_check.rb" > "$TMP"
run "pipeline_check (fixture_fan)" "$TMP"
rm -f "$TMP"

echo "--- all suites passed"
