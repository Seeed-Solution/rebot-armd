#!/usr/bin/env python3
"""Self-tests for scripts/check_source_limits using temporary git repositories."""

from __future__ import annotations

import json
import subprocess
import sys
import tempfile
import unittest
from pathlib import Path

SCRIPT = Path(__file__).resolve().parent.parent / "check_source_limits"


def run(cmd: list[str], cwd: Path | None = None) -> subprocess.CompletedProcess[str]:
    return subprocess.run(cmd, cwd=cwd, text=True, capture_output=True)


def make_lines(n: int) -> str:
    return "".join(f"// line {i}\n" for i in range(1, n + 1))


class CheckSourceLimitsTest(unittest.TestCase):
    def setUp(self) -> None:
        self._tmp = tempfile.TemporaryDirectory()
        self.repo = Path(self._tmp.name)
        self.git("init", "-q", "-b", "main")
        self.git("config", "user.email", "test@example.com")
        self.git("config", "user.name", "Test")
        (self.repo / "src").mkdir()
        (self.repo / "src" / "vendor").mkdir()
        (self.repo / "src" / "small.rs").write_text(make_lines(100), encoding="utf-8")
        (self.repo / "src" / "warn.rs").write_text(make_lines(1800), encoding="utf-8")
        (self.repo / "src" / "big.rs").write_text(make_lines(3500), encoding="utf-8")
        (self.repo / "src" / "vendor" / "lib.min.js").write_text(make_lines(4200), encoding="utf-8")
        self.git("add", ".")
        self.git("commit", "-qm", "base files")

        # Initialize allowlist on main.
        res = self.check("--init")
        self.assertEqual(res.returncode, 0, res.stderr)
        self.git("add", ".config/source_limits_allowlist.json")
        self.git("commit", "-qm", "add allowlist")
        self.git("checkout", "-qb", "feature")

    def tearDown(self) -> None:
        self._tmp.cleanup()

    def git(self, *args: str) -> subprocess.CompletedProcess[str]:
        res = run(["git", "-C", str(self.repo), *args])
        self.assertEqual(res.returncode, 0, f"git {args} failed: {res.stderr}")
        return res

    def check(self, *args: str) -> subprocess.CompletedProcess[str]:
        return run([sys.executable, str(SCRIPT), "--root", str(self.repo), *args])

    def test_baseline_passes_and_warns_without_blocking(self) -> None:
        res = self.check("--base", "main")
        self.assertEqual(res.returncode, 0, res.stderr)
        self.assertIn("WARN   1800  src/warn.rs", res.stdout)
        self.assertIn("source_limits: PASS", res.stdout)
        self.assertIn("allowlisted_gt3000=1", res.stdout)
        # Vendor file must be ignored.
        data = json.loads((self.repo / ".config/source_limits_allowlist.json").read_text())
        self.assertEqual(data["files"], {"src/big.rs": 3500})

    def test_new_file_over_3000_blocks(self) -> None:
        (self.repo / "src" / "new_huge.rs").write_text(make_lines(3001), encoding="utf-8")
        res = self.check("--base", "main")
        self.assertEqual(res.returncode, 1)
        self.assertIn("OVER LIMIT: src/new_huge.rs", res.stderr)

    def test_adding_entry_to_allowlist_blocks(self) -> None:
        (self.repo / "src" / "new_huge.rs").write_text(make_lines(3005), encoding="utf-8")
        p = self.repo / ".config/source_limits_allowlist.json"
        data = json.loads(p.read_text())
        data["files"]["src/new_huge.rs"] = 3005
        p.write_text(json.dumps(data, indent=2) + "\n")
        res = self.check("--base", "main")
        self.assertEqual(res.returncode, 1)
        self.assertIn("ALLOWLIST ADD: src/new_huge.rs", res.stderr)

    def test_raising_allowlist_cap_blocks(self) -> None:
        (self.repo / "src" / "big.rs").write_text(make_lines(3510), encoding="utf-8")
        p = self.repo / ".config/source_limits_allowlist.json"
        data = json.loads(p.read_text())
        data["files"]["src/big.rs"] = 3510
        p.write_text(json.dumps(data, indent=2) + "\n")
        res = self.check("--base", "main")
        self.assertEqual(res.returncode, 1)
        self.assertIn("ALLOWLIST GROW: src/big.rs", res.stderr)
        self.assertIn("GREW ABOVE BASE: src/big.rs", res.stderr)

    def test_growing_allowlisted_file_above_merge_base_blocks_even_under_cap(self) -> None:
        # Simulate a stale higher cap on main (3500) where big.rs was already shrunk to 3400 on main.
        self.git("checkout", "-q", "main")
        (self.repo / "src" / "big.rs").write_text(make_lines(3400), encoding="utf-8")
        self.git("add", "src/big.rs")
        self.git("commit", "-qm", "shrink big.rs without touching allowlist")
        self.git("checkout", "-qb", "grow-from-3400")
        (self.repo / "src" / "big.rs").write_text(make_lines(3405), encoding="utf-8")
        res = self.check("--base", "main")
        self.assertEqual(res.returncode, 1)
        self.assertIn("GREW ABOVE BASE: src/big.rs", res.stderr)

    def test_shrinking_below_3000_requires_removing_stale_entry(self) -> None:
        (self.repo / "src" / "big.rs").write_text(make_lines(2900), encoding="utf-8")
        res = self.check("--base", "main")
        self.assertEqual(res.returncode, 1)
        self.assertIn("STALE ALLOWLIST: src/big.rs", res.stderr)

        upd = self.check("--update")
        self.assertEqual(upd.returncode, 0, upd.stderr)
        res2 = self.check("--base", "main")
        self.assertEqual(res2.returncode, 0, res2.stderr)
        data = json.loads((self.repo / ".config/source_limits_allowlist.json").read_text())
        self.assertEqual(data["files"], {})

    def test_shrinking_still_over_3000_passes_and_update_ratchets(self) -> None:
        (self.repo / "src" / "big.rs").write_text(make_lines(3200), encoding="utf-8")
        res = self.check("--base", "main")
        self.assertEqual(res.returncode, 0, res.stderr)
        self.assertIn("shrunk 3500 -> 3200", res.stdout)

        upd = self.check("--update")
        self.assertEqual(upd.returncode, 0, upd.stderr)
        data = json.loads((self.repo / ".config/source_limits_allowlist.json").read_text())
        self.assertEqual(data["files"]["src/big.rs"], 3200)
        res2 = self.check("--base", "main")
        self.assertEqual(res2.returncode, 0, res2.stderr)

    def test_raising_thresholds_in_config_blocks(self) -> None:
        p = self.repo / ".config/source_limits_allowlist.json"
        data = json.loads(p.read_text())
        data["block_lines"] = 9999
        p.write_text(json.dumps(data, indent=2) + "\n")
        res = self.check("--base", "main")
        self.assertEqual(res.returncode, 1)
        self.assertIn("thresholds must be", res.stderr)

    def test_update_never_adds_new_files(self) -> None:
        (self.repo / "src" / "sneaky.rs").write_text(make_lines(3100), encoding="utf-8")
        upd = self.check("--update")
        self.assertEqual(upd.returncode, 0)
        data = json.loads((self.repo / ".config/source_limits_allowlist.json").read_text())
        self.assertNotIn("src/sneaky.rs", data["files"])
        res = self.check("--base", "main")
        self.assertEqual(res.returncode, 1)

    def test_checks_cjs_py_and_extensionless_scripts(self) -> None:
        (self.repo / "scripts").mkdir()
        (self.repo / "scripts" / "huge.cjs").write_text(make_lines(3001), encoding="utf-8")
        (self.repo / "scripts" / "huge.py").write_text(make_lines(3002), encoding="utf-8")
        (self.repo / "scripts" / "tool_bin").write_text("#!/usr/bin/env python3\n" + make_lines(3001), encoding="utf-8")
        res = self.check("--base", "main")
        self.assertEqual(res.returncode, 1)
        self.assertIn("OVER LIMIT: scripts/huge.cjs", res.stderr)
        self.assertIn("OVER LIMIT: scripts/huge.py", res.stderr)
        self.assertIn("OVER LIMIT: scripts/tool_bin", res.stderr)

    def test_missing_default_base_fails_closed(self) -> None:
        # Rename main away so neither origin/main nor main exists.
        self.git("branch", "-m", "main", "other-base")
        res = self.check()
        self.assertEqual(res.returncode, 2)
        self.assertIn("cannot resolve a merge-base comparison ref", res.stderr)


if __name__ == "__main__":
    unittest.main()
