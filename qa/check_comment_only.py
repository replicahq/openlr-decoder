"""Check that changes to Rust files since a git ref touch only comments.

Strips `//` comments and blank lines from each changed .rs file, old and new,
and diffs the results.

    python qa/check_comment_only.py          # against main
    python qa/check_comment_only.py HEAD~1
"""

import difflib
import subprocess
import sys


def strip(text: str) -> list[str]:
    out = []
    for line in text.splitlines():
        kept, in_str, i = [], False, 0
        while i < len(line):
            c = line[i]
            if c == '"' and (i == 0 or line[i - 1] != "\\"):
                in_str = not in_str
            if not in_str and line.startswith("//", i):
                break
            kept.append(c)
            i += 1
        s = "".join(kept).rstrip()
        if s.strip():
            out.append(s)
    return out


def git(*args: str) -> str:
    return subprocess.run(["git", *args], capture_output=True, text=True, check=True).stdout


def main() -> int:
    ref = sys.argv[1] if len(sys.argv) > 1 else "main"
    files = [f for f in git("diff", "--name-only", ref, "--", "*.rs").split() if f]
    failed = False
    for path in files:
        try:
            old = git("show", f"{ref}:{path}")
        except subprocess.CalledProcessError:
            old = ""
        try:
            new = open(path).read()
        except FileNotFoundError:
            new = ""
        diff = list(difflib.unified_diff(strip(old), strip(new), path, path, lineterm=""))
        if diff:
            failed = True
            print("\n".join(diff))
        else:
            print(f"ok  {path}")
    return 1 if failed else 0


if __name__ == "__main__":
    sys.exit(main())
