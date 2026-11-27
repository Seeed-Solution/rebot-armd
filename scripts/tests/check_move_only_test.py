#!/usr/bin/env python3
"""Self-tests for scripts/check_move_only using temporary git repositories."""

from __future__ import annotations

import subprocess
import sys
import tempfile
import unittest
from pathlib import Path

SCRIPT = Path(__file__).resolve().parent.parent / "check_move_only"


def run(cmd: list[str], cwd: Path | None = None) -> subprocess.CompletedProcess[str]:
    return subprocess.run(cmd, cwd=cwd, text=True, capture_output=True)


class CheckMoveOnlyTest(unittest.TestCase):
    def setUp(self) -> None:
        self._tmp = tempfile.TemporaryDirectory()
        self.repo = Path(self._tmp.name)
        self.git("init", "-q", "-b", "main")
        self.git("config", "user.email", "test@example.com")
        self.git("config", "user.name", "Test")
        (self.repo / "src").mkdir()
        (self.repo / "src" / "driver.rs").write_text(
            "pub struct Driver<C> {\n"
            "    transport: C,\n"
            "    counter: u32,\n"
            "}\n"
            "\n"
            "impl<C> Driver<C> {\n"
            "    fn helper_one(&mut self, delta: u32) -> u32 {\n"
            "        let next = self.counter.saturating_add(delta);\n"
            "        self.counter = next;\n"
            "        next\n"
            "    }\n"
            "\n"
            "    fn helper_short(&self) -> u32 {\n"
            "        self.counter\n"
            "    }\n"
            "}\n"
            "\n"
            "mod emitter {\n"
            "    pub(super) fn emit(val: u32) -> u32 {\n"
            "        let clamped = val.min(100);\n"
            "        clamped * 2\n"
            "    }\n"
            "}\n",
            encoding="utf-8",
        )
        self.git("add", ".")
        self.git("commit", "-qm", "initial driver")

    def tearDown(self) -> None:
        self._tmp.cleanup()

    def git(self, *args: str) -> subprocess.CompletedProcess[str]:
        res = run(["git", "-C", str(self.repo), *args])
        self.assertEqual(res.returncode, 0, f"git {args} failed: {res.stderr}")
        return res

    def check(self, *args: str) -> subprocess.CompletedProcess[str]:
        return run([sys.executable, str(SCRIPT), "--root", str(self.repo), *args])

    def test_cross_file_split_with_vis_impl_mod_use_and_dedent_passes(self) -> None:
        (self.repo / "src" / "driver").mkdir()
        (self.repo / "src" / "driver.rs").write_text(
            "mod helpers;\n"
            "mod emitter;\n"
            "\n"
            "pub struct Driver<C> {\n"
            "    pub(super) transport: C,\n"
            "    pub(super) counter: u32,\n"
            "}\n",
            encoding="utf-8",
        )
        (self.repo / "src" / "driver" / "helpers.rs").write_text(
            "//! Extracted driver helpers.\n"
            "use super::{\n"
            "    Driver,\n"
            "};\n"
            "\n"
            "impl<C> Driver<C>\n"
            "where\n"
            "    C: Sized,\n"
            "{\n"
            "    pub(super) fn helper_one(&mut self, delta: u32) -> u32 {\n"
            "        let next = self.counter.saturating_add(delta);\n"
            "        self.counter = next;\n"
            "        next\n"
            "    }\n"
            "\n"
            "    pub(super) fn helper_short(&self) -> u32 {\n"
            "        self.counter\n"
            "    }\n"
            "}\n",
            encoding="utf-8",
        )
        (self.repo / "src" / "driver" / "emitter.rs").write_text(
            "pub(super) fn emit(val: u32) -> u32 {\n"
            "    let clamped = val.min(100);\n"
            "    clamped * 2\n"
            "}\n",
            encoding="utf-8",
        )
        self.git("add", ".")
        res = self.check("--staged")
        # Note: `where\n    C: Sized,\n{` adds a new bound inside impl header -> allowed as impl header.
        # Wait: removing `impl<C> Driver<C> {` and `}` in driver.rs is also balanced!
        self.assertEqual(res.returncode, 0, f"stdout={res.stdout}\nstderr={res.stderr}")
        self.assertIn("check_move_only: PASS", res.stdout)

    def test_pure_git_mv_passes(self) -> None:
        self.git("mv", "src/driver.rs", "src/driver_moved.rs")
        res = self.check("--staged")
        self.assertEqual(res.returncode, 0, res.stderr)
        self.assertIn("non_move=0", res.stdout)

    def test_logic_edit_during_move_fails(self) -> None:
        (self.repo / "src" / "emitter.rs").write_text(
            "pub(super) fn emit(val: u32) -> u32 {\n"
            "    let clamped = val.min(200);\n"
            "    clamped * 2\n"
            "}\n",
            encoding="utf-8",
        )
        orig = (self.repo / "src" / "driver.rs").read_text(encoding="utf-8")
        cut = orig[: orig.index("mod emitter {")] + "mod emitter;\n"
        (self.repo / "src" / "driver.rs").write_text(cut, encoding="utf-8")
        self.git("add", ".")
        res = self.check("--staged")
        self.assertEqual(res.returncode, 1, res.stdout)
        self.assertIn("clamped = val.min(100)", res.stderr)
        self.assertIn("clamped = val.min(200)", res.stderr)

    def test_deleted_statement_fails(self) -> None:
        orig = (self.repo / "src" / "driver.rs").read_text(encoding="utf-8")
        (self.repo / "src" / "driver.rs").write_text(
            orig.replace("        self.counter = next;\n", ""), encoding="utf-8"
        )
        res = self.check()
        self.assertEqual(res.returncode, 1)
        self.assertIn("self.counter = next;", res.stderr)

    def test_unsafe_impl_rejected(self) -> None:
        (self.repo / "src" / "extra.rs").write_text(
            "unsafe impl<C> Send for Driver<C> {\n}\n", encoding="utf-8"
        )
        self.git("add", ".")
        res = self.check("--staged")
        self.assertEqual(res.returncode, 1)
        self.assertIn("unsafe impl", res.stderr)

    def test_unbalanced_brace_rejected(self) -> None:
        (self.repo / "src" / "extra.rs").write_text("}\n", encoding="utf-8")
        self.git("add", ".")
        res = self.check("--staged")
        self.assertEqual(res.returncode, 1)
        self.assertIn("NON-MOVE", res.stderr)

    def test_same_file_statement_swap_rejected(self) -> None:
        (self.repo / "src" / "swap.rs").write_text(
            "fn f() {\n"
            "    alpha();\n"
            "    beta();\n"
            "}\n",
            encoding="utf-8",
        )
        self.git("add", "src/swap.rs")
        self.git("commit", "-qm", "add swap.rs")
        (self.repo / "src" / "swap.rs").write_text(
            "fn f() {\n"
            "    beta();\n"
            "    alpha();\n"
            "}\n",
            encoding="utf-8",
        )
        res = self.check()
        self.assertEqual(res.returncode, 1, f"stdout={res.stdout}\nstderr={res.stderr}")

    def test_cfg_test_mod_declaration_passes(self) -> None:
        orig = (self.repo / "src" / "driver.rs").read_text(encoding="utf-8")
        (self.repo / "src" / "driver.rs").write_text(
            orig + "\n#[cfg(test)]\nmod tests;\n", encoding="utf-8"
        )
        res = self.check()
        self.assertEqual(res.returncode, 0, res.stderr)


if __name__ == "__main__":
    unittest.main()
