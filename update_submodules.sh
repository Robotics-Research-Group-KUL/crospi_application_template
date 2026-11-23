#!/bin/bash

#  Copyright (c) 2025 KU Leuven, Belgium
#
#  Author: Santiago Iregui
#  email: <santiago.iregui@kuleuven.be>
#
#  GNU Lesser General Public License Usage
#  Alternatively, this file may be used under the terms of the GNU Lesser
#  General Public License version 3 as published by the Free Software
#  Foundation and appearing in the file LICENSE.LGPLv3 included in the
#  packaging of this file. Please review the following information to
#  ensure the GNU Lesser General Public License version 3 requirements
#  will be met: https://www.gnu.org/licenses/lgpl.html.
#
#  This program is distributed in the hope that it will be useful,
#  but WITHOUT ANY WARRANTY; without even the implied warranty of
#  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
#  GNU Lesser General Public License for more details.

#
# Checks out the main branch and pulls the latest changes of all submodules
# (task libraries, skill libraries, ...) recursively.
#
# Usage:
#   ./update_submodules.sh            # pull this repository, then all submodules
#   ./update_submodules.sh --no-self  # update submodules only
#   ./update_submodules.sh -b <name>  # use another default branch (default: main)
#
# A submodule's branch can also be set per submodule in .gitmodules
# (e.g. `branch = develop`), which takes precedence over the default.
#

set -e

bold=$(tput bold 2>/dev/null || true)
normal=$(tput sgr0 2>/dev/null || true)
RED='\e[38;5;198m'
GREEN='\e[32m'
DEFAULTC='\e[39m'

DEFAULT_BRANCH="main"
PULL_SELF=true

while [[ $# -gt 0 ]]; do
    case "$1" in
        --no-self) PULL_SELF=false; shift ;;
        -b|--branch) DEFAULT_BRANCH="$2"; shift 2 ;;
        -h|--help) sed -n '21,32p' "$0"; exit 0 ;;
        *) echo -e "${RED}Unknown argument: $1${DEFAULTC}"; exit 1 ;;
    esac
done

# Always operate from the repository root, regardless of where the script is called from
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "$SCRIPT_DIR"

if [ "$PULL_SELF" = true ]; then
    echo -e "${bold}Pulling $(basename "$SCRIPT_DIR")...${normal}"
    git pull --ff-only
fi

echo -e "${bold}Initializing submodules...${normal}"
git submodule sync --recursive
git submodule update --init --recursive

echo -e "${bold}Checking out '${DEFAULT_BRANCH}' and pulling in every submodule...${normal}"
# `foreach --recursive` visits parents before their nested submodules, so nested
# ones are updated after their parent moved to the latest commit.
export DEFAULT_BRANCH RED GREEN DEFAULTC
git submodule foreach --recursive '
    branch=$(git config -f "$toplevel/.gitmodules" "submodule.$name.branch" || echo "$DEFAULT_BRANCH")
    git fetch --quiet origin
    git checkout "$branch"
    git pull --ff-only origin "$branch"
    git submodule update --init --recursive
    printf "%b  -> %s is now at %s on %s%b\n" "$GREEN" "$displaypath" "$(git rev-parse --short HEAD)" "$branch" "$DEFAULTC"
'

echo ""
echo -e "${bold}Done.${normal} Submodule pointers may now differ from the commits recorded in this repository:"
git status --short
