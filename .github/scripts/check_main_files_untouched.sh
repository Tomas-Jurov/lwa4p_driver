#!/usr/bin/env bash
# Fails when a revision changes, deletes or renames a file that comes from main
# (Tomas-Jurov/lwa4p_driver). Only added files are allowed. See AGENTS.md, section 2.
#
# Usage: check_main_files_untouched.sh [MAIN_REF] [HEAD_REF]
#   MAIN_REF  defaults to origin/main
#   HEAD_REF  defaults to HEAD
#
# HEAD_REF is compared with the last MAIN_REF commit it already contains (the
# merge base), not with the tip of MAIN_REF. Between "Sync fork" and the
# main -> devel merge, main is ahead of devel, and a comparison with its tip
# would flag Tomas's own new changes.

set -euo pipefail

main_ref="${1:-origin/main}"
head_ref="${2:-HEAD}"

if ! base="$(git merge-base "${main_ref}" "${head_ref}")"; then
  echo "error: ${head_ref} shares no history with ${main_ref}." >&2
  exit 2
fi

echo "Checking ${head_ref} against the last ${main_ref} commit it contains:"
echo "  $(git log -1 --format='%h %s' "${base}")"

# --no-renames reports a moved file as deleted + added, so the deletion is caught.
violations="$(git diff --name-status --no-renames --diff-filter=DMT "${base}" "${head_ref}")"

if [[ -z "${violations}" ]]; then
  echo "OK: only added files. Every file from ${main_ref} is unchanged."
  exit 0
fi

echo
echo "These files come from Tomas-Jurov/lwa4p_driver and must stay unchanged in devel:"
while IFS=$'\t' read -r status path; do
  case "${status}" in
    D) reason="deleted or moved" ;;
    T) reason="type changed" ;;
    *) reason="modified" ;;
  esac
  echo "  ${reason}: ${path}"
  if [[ -n "${GITHUB_ACTIONS:-}" ]]; then
    echo "::error file=${path}::${path} comes from Tomas's main and was ${reason}."
  fi
done <<< "${violations}"
echo
echo "Revert these changes. If the driver or the model must change, propose it to"
echo "Tomas from a branch based on main (AGENTS.md, section 2, workflow 3)."
exit 1
