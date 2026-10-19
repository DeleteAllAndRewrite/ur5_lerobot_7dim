import sys
import types

import pytest

from ur5_capture.record import TerminalKeys


def windows_console(monkeypatch, chars):
    chars = list(chars)
    monkeypatch.setattr(sys, "platform", "win32")
    monkeypatch.setattr(sys.stdin, "isatty", lambda: True)
    monkeypatch.setitem(sys.modules, "msvcrt", types.SimpleNamespace(
        kbhit=lambda: bool(chars), getwch=lambda: chars.pop(0)))


def test_windows_g_and_episode_keys(monkeypatch):
    windows_console(monkeypatch, "Gsdq")
    with TerminalKeys() as keys:
        assert [keys.read() for _ in range(5)] == ["g", "s", "d", "q", ""]


def test_windows_special_keys_and_ctrl_c(monkeypatch):
    windows_console(monkeypatch, ["\xe0", "K", "\x03"])
    with TerminalKeys() as keys:
        assert keys.read() == ""
        with pytest.raises(KeyboardInterrupt):
            keys.read()


def test_next_dataset_root_numbers_from_0000(tmp_path):
    from ur5_capture.record import next_dataset_root
    base = tmp_path / "ur5_three_rgb"
    assert next_dataset_root(str(base)) == tmp_path / "ur5_three_rgb_0000"
    base.mkdir()  # legacy unnumbered dataset is ignored and kept
    (tmp_path / "ur5_three_rgb_0000").mkdir()
    (tmp_path / "ur5_three_rgb_0007").mkdir()
    (tmp_path / "ur5_three_rgb_backup").mkdir()
    (tmp_path / "ur5_three_rgb_extra_0009").mkdir()
    assert next_dataset_root(str(base)) == tmp_path / "ur5_three_rgb_0008"
