# Third-party sources

`lerobot/` (a pinned Git submodule integrated from the local `lerobot-rerun` checkout)
and `compat/lerobot_v21/` (a source snapshot) contain unmodified sources from
Hugging Face LeRobot, https://github.com/huggingface/lerobot, licensed under
Apache License 2.0. Copyright notices remain in the source files; the original
license is included in each directory. Exact commits are recorded in UPSTREAM.json.

The v2.1 snapshot includes source and installation metadata without Git history
or upstream media. The v3 submodule retains its upstream repository structure.
Rerun SDK is installed as a runtime dependency under its own license;
https://github.com/rerun-io/rerun. Runtime dependencies retain their respective licenses.
