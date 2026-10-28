# AGENTS.md — oxijolt

Rules for coding agents working in this repository. The project's rules for every contributor live in
[CONTRIBUTING.md](CONTRIBUTING.md): read it fully before changing anything; it is binding. `CLAUDE.md`
includes both files.

On top of it, agents:
- do not publish, tag, create GitHub releases or merge `release/v*` branches; the owner does (see
  "Releases" and "Publishing" in CONTRIBUTING.md);
- run one cargo process at a time: the C++ build is heavy and the machines are shared;
- add no AI attribution lines to commits or pull requests.
