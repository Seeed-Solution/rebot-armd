#!/usr/bin/env python3
"""Self-tests for scripts/check_move_only using temporary git repositories."""

from __future__ import annotations

import os
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
            "use std::fmt;\n"
            "\n"
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
            "use std::fmt;\n"
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

    def test_deleted_statement_fails(self) -> None:
        orig = (self.repo / "src" / "driver.rs").read_text(encoding="utf-8")
        (self.repo / "src" / "driver.rs").write_text(
            orig.replace("        self.counter = next;\n", ""), encoding="utf-8"
        )
        res = self.check()
        self.assertEqual(res.returncode, 1)
        self.assertIn("self.counter = next;", res.stderr)

    def test_use_line_with_trailing_statement_rejected(self) -> None:
        orig = (self.repo / "src" / "driver.rs").read_text(encoding="utf-8")
        (self.repo / "src" / "driver.rs").write_text(
            orig + "\nuse std::io; do_something();\n", encoding="utf-8"
        )
        res = self.check()
        self.assertEqual(res.returncode, 1, f"stdout={res.stdout}\nstderr={res.stderr}")
        self.assertIn("do_something();", res.stderr)

    def test_deleted_use_without_relocation_rejected(self) -> None:
        orig = (self.repo / "src" / "driver.rs").read_text(encoding="utf-8")
        (self.repo / "src" / "driver.rs").write_text(
            orig.replace("use std::fmt;\n", ""), encoding="utf-8"
        )
        res = self.check()
        self.assertEqual(res.returncode, 1, f"stdout={res.stdout}\nstderr={res.stderr}")
        self.assertIn("use std::fmt;", res.stderr)

    def test_cross_file_statement_exchange_rejected(self) -> None:
        (self.repo / "src" / "a.rs").write_text("fn a() {\n    start();\n}\n", encoding="utf-8")
        (self.repo / "src" / "b.rs").write_text("fn b() {\n    stop();\n}\n", encoding="utf-8")
        self.git("add", "src/a.rs", "src/b.rs")
        self.git("commit", "-qm", "add a and b")
        (self.repo / "src" / "a.rs").write_text("fn a() {\n    stop();\n}\n", encoding="utf-8")
        (self.repo / "src" / "b.rs").write_text("fn b() {\n    start();\n}\n", encoding="utf-8")
        res = self.check()
        self.assertEqual(res.returncode, 1, f"stdout={res.stdout}\nstderr={res.stderr}")

    def test_same_file_long_statement_reorder_rejected(self) -> None:
        (self.repo / "src" / "order.rs").write_text(
            "fn run() {\n"
            "    first_long_safety_check_step();\n"
            "    second_long_safety_check_step();\n"
            "}\n",
            encoding="utf-8",
        )
        self.git("add", "src/order.rs")
        self.git("commit", "-qm", "add order.rs")
        (self.repo / "src" / "order.rs").write_text(
            "fn run() {\n"
            "    second_long_safety_check_step();\n"
            "    first_long_safety_check_step();\n"
            "}\n",
            encoding="utf-8",
        )
        res = self.check()
        self.assertEqual(res.returncode, 1, f"stdout={res.stdout}\nstderr={res.stderr}")

    def test_multiline_raw_string_indent_change_rejected(self) -> None:
        (self.repo / "src" / "raw.rs").write_text(
            'const SQL: &str = r#"\n    SELECT 1\n"#;\n', encoding="utf-8"
        )
        self.git("add", "src/raw.rs")
        self.git("commit", "-qm", "add raw.rs")
        (self.repo / "src" / "raw.rs").write_text(
            'const SQL: &str = r#"\n  SELECT 1\n"#;\n', encoding="utf-8"
        )
        res = self.check()
        self.assertEqual(res.returncode, 1, f"stdout={res.stdout}\nstderr={res.stderr}")
        self.assertIn("SELECT 1", res.stderr)

    def test_mode_only_change_rejected(self) -> None:
        script = self.repo / "src" / "tool.sh"
        script.write_text("#!/bin/sh\necho hi\n", encoding="utf-8")
        os.chmod(script, 0o755)
        self.git("add", "src/tool.sh")
        self.git("commit", "-qm", "add executable script")
        os.chmod(script, 0o644)
        res = self.check()
        self.assertEqual(res.returncode, 1, f"stdout={res.stdout}\nstderr={res.stderr}")
        self.assertIn("MODE CHANGE", res.stderr)

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

    def test_cfg_test_mod_declaration_passes(self) -> None:
        orig = (self.repo / "src" / "driver.rs").read_text(encoding="utf-8")
        (self.repo / "src" / "driver.rs").write_text(
            orig + "\n#[cfg(test)]\nmod tests;\n", encoding="utf-8"
        )
        res = self.check()
        self.assertEqual(res.returncode, 0, res.stderr)

    def test_malformed_impl_followed_by_fn_rejected(self) -> None:
        (self.repo / "src" / "bad.rs").write_text(
            "impl Foo\nfn sneaky() {\n}\n", encoding="utf-8"
        )
        self.git("add", ".")
        res = self.check("--staged")
        self.assertEqual(res.returncode, 1)
        self.assertIn("fn sneaky()", res.stderr)


if __name__ == "__main__":
    unittest.main()
