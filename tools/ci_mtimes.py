#!/usr/bin/env python3
"""
CI's build cache: make the files that did not change since the cached build look old.

A fresh checkout gives every file the time of the checkout, so cargo and make take every
source as changed and rebuild all of our crates and packages, cache or not. The cache
(build/ and install/, .github/workflows/test.yml) keeps the commit it was built from in
build/.aav-cache-commit. This script asks git which files differ between that commit and
HEAD, and sets every other tracked file to an old time (2001): older than anything in
the cache, so cargo and make skip them. The files that differ keep the checkout's time,
newer than the cache, so they are rebuilt.

    python3 tools/ci_mtimes.py restore     after the cache is restored, before the build
    python3 tools/ci_mtimes.py record      before the cache is saved: write the commit

Anything unsure (no commit file, a commit git can't find, git failing) changes nothing:
the build is then a full one, as without this script. A fault never makes a stale build.
"""

import os
import subprocess
import sys

COMMIT_FILE = os.path.join('build', '.aav-cache-commit')
OLD = 978307200   # 2001-01-01 00:00 UTC: older than any cache


def unchanged(tracked, changed):
    """Return the tracked files that are not among the changed ones."""
    changed = set(changed)
    return [f for f in tracked if f and f not in changed]


def git(*args):
    return subprocess.run(['git', *args], check=True, capture_output=True,
                          text=True).stdout


def restore():
    """Make the files unchanged since the cache's commit old; say how many of each."""
    try:
        with open(COMMIT_FILE, encoding='utf-8') as f:
            commit = f.read().strip()
    except OSError:
        print('ci_mtimes: no cache commit: every file stays new (a full build)')
        return 0
    try:
        # A shallow checkout may not have it: fetch that one commit (allowed on GitHub).
        if subprocess.run(['git', 'cat-file', '-e', f'{commit}^{{commit}}'],
                          capture_output=True).returncode != 0:
            git('fetch', '-q', '--depth=1', 'origin', commit)
        changed = git('diff', '--no-renames', '--name-only', '-z', commit, 'HEAD').split('\0')
        tracked = git('ls-files', '-z').split('\0')
    except (subprocess.CalledProcessError, OSError) as e:
        print(f'ci_mtimes: git could not compare with {commit[:12]} ({e}): a full build')
        return 0
    old = unchanged(tracked, changed)
    for path in old:
        try:
            os.utime(path, (OLD, OLD), follow_symlinks=False)
        except OSError:
            pass   # a submodule or a file the checkout left out
    print(f'ci_mtimes: cache from {commit[:12]}; {len(old)} files unchanged (made old), '
          f'{len([c for c in changed if c])} changed (rebuilt)')
    return 0


def record():
    """Write HEAD into the cache, so the next run knows what it was built from."""
    os.makedirs(os.path.dirname(COMMIT_FILE), exist_ok=True)
    with open(COMMIT_FILE, 'w', encoding='utf-8') as f:
        f.write(git('rev-parse', 'HEAD'))
    return 0


def main(argv):
    if argv[1:] == ['restore']:
        return restore()
    if argv[1:] == ['record']:
        return record()
    print(__doc__.strip().split('\n\n')[2], file=sys.stderr)
    return 2


if __name__ == '__main__':
    sys.exit(main(sys.argv))
