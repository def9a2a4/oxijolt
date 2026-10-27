#!/usr/bin/env bash
# Checks that every object of the given static libraries is thin arm64 macOS code built for
# exactly the given minimum macOS version.
#
# Usage: bash scripts/check_macos_objects.sh <minimum> <library>...
#
# A release archive promises a minimum macOS (`macos:` in PROVENANCE.txt). An object built for
# a newer minimum than Rust's for the target would make the archive refuse or misbehave for
# older deployment targets, so the release audits each object's LC_BUILD_VERSION command
# (platform and `minos`, never the SDK) before it writes that promise. An object with no such
# command, with two, or with an older LC_VERSION_MIN_* command fails the check.
# Requires Bash 3.2 or newer and the Xcode command line tools (lipo, ar, otool).
set -euo pipefail

minimum=${1:?usage: check_macos_objects.sh <minimum> <library>...}
shift
[ $# -ge 1 ] || { echo "usage: check_macos_objects.sh <minimum> <library>..."; exit 1; }

failed=0
for lib in "$@"; do
  archs=$(lipo -archs "$lib")
  if [ "$archs" != arm64 ]; then
    echo "$lib: architectures '$archs', expected arm64 only"
    failed=1
    continue
  fi
  listing=$(ar -t "$lib")
  members=$(printf '%s\n' "$listing" | grep -cv '^__\.SYMDEF' || true)
  if [ "$members" -lt 1 ]; then
    echo "$lib: no object files"
    failed=1
    continue
  fi
  commands=$(otool -l "$lib")
  # One `lib.a(member.o):` line starts each member; prints "<members seen> <members good>".
  read -r headers good < <(printf '%s\n' "$commands" | awk -v want="$minimum" '
    function finish() {
      if (member == "") return
      headers++
      if (builds == 1 && !old && (platform == "1" || tolower(platform) == "macos") && minos == want) {
        good++
      } else {
        printf "  %s: %d LC_BUILD_VERSION, platform %s, minos %s%s\n", member, builds, platform, minos,
          (old ? ", LC_VERSION_MIN" : "") > "/dev/stderr"
      }
    }
    /\(.*\):$/ { finish(); member = $0; builds = 0; old = 0; platform = ""; minos = ""; in_build = 0; next }
    $1 == "cmd" {
      in_build = ($2 == "LC_BUILD_VERSION")
      if (in_build) builds++
      if ($2 ~ /^LC_VERSION_MIN_/) old = 1
      next
    }
    in_build && $1 == "platform" { platform = $2 }
    in_build && $1 == "minos" { minos = $2 }
    END { finish(); print headers + 0, good + 0 }')
  echo "$lib: $members objects, $headers with load commands, $good for macOS $minimum"
  if [ "$headers" -ne "$members" ] || [ "$good" -ne "$members" ]; then
    echo "$lib: not every object is built for macOS $minimum"
    failed=1
  fi
done
[ "$failed" -eq 0 ]
