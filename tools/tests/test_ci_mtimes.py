"""tools/ci_mtimes.py: unchanged files look old to the build, changed ones new."""

import os
import subprocess
import sys

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))

import ci_mtimes  # noqa: E402

SCRIPT = os.path.join(os.path.dirname(__file__), '..', 'ci_mtimes.py')


def git(repo, *args):
    subprocess.run(['git', '-c', 'user.name=t', '-c', 'user.email=t@t', *args],
                   cwd=repo, check=True, capture_output=True)


def run_in(repo, word):
    return subprocess.run([sys.executable, SCRIPT, word], cwd=repo, capture_output=True,
                          text=True, check=True).stdout


def make_repo(tmp_path):
    repo = tmp_path / 'repo'
    repo.mkdir()
    git(repo, 'init', '-q')
    git(repo, 'remote', 'add', 'origin', str(repo))
    (repo / 'a.rs').write_text('fn a() {}\n')
    (repo / 'b.rs').write_text('fn b() {}\n')
    (repo / '.gitignore').write_text('build/\n')    # as ours: the cache is never in git
    git(repo, 'add', '.')
    git(repo, 'commit', '-q', '-m', 'cached')
    return repo


def test_unchanged_leaves_out_the_changed_and_the_empty():
    assert ci_mtimes.unchanged(['a', 'b', '', 'c'], ['b', '']) == ['a', 'c']


def test_only_the_files_changed_since_the_cache_stay_new(tmp_path):
    repo = make_repo(tmp_path)
    run_in(repo, 'record')
    assert (repo / 'build' / '.aav-cache-commit').read_text().strip()
    (repo / 'b.rs').write_text('fn b() { 1; }\n')
    (repo / 'c.rs').write_text('fn c() {}\n')
    git(repo, 'add', '.')
    git(repo, 'commit', '-q', '-m', 'the change')
    out = run_in(repo, 'restore')
    assert '2 files unchanged' in out and '2 changed' in out
    assert os.stat(repo / 'a.rs').st_mtime == ci_mtimes.OLD
    assert os.stat(repo / 'b.rs').st_mtime > ci_mtimes.OLD
    assert os.stat(repo / 'c.rs').st_mtime > ci_mtimes.OLD


def test_without_a_known_commit_nothing_changes(tmp_path):
    repo = make_repo(tmp_path)
    before = os.stat(repo / 'a.rs').st_mtime
    assert 'full build' in run_in(repo, 'restore')
    (repo / 'build').mkdir()
    (repo / 'build' / '.aav-cache-commit').write_text('0' * 40)
    assert 'full build' in run_in(repo, 'restore')
    assert os.stat(repo / 'a.rs').st_mtime == before


def test_a_wrong_word_lists_the_usage():
    assert ci_mtimes.main(['ci_mtimes.py']) == 2
    assert ci_mtimes.main(['ci_mtimes.py', 'save']) == 2
