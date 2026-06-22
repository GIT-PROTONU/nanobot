---
name: memory-in-git
description: "User wants Claude memory version-controlled in the repo, always"
metadata: 
  node_type: memory
  type: feedback
  originSessionId: 9c559fa2-8d06-4b5a-aaf1-f9c937da8fbf
---

The user **always wants memory committed to git**, not just in the local `~/.claude` store.

**Why:** so the knowledge is versioned, shared, and survives a machine wipe.

**How to apply:** the canonical memory dir is the repo's **`.claude/memory/`** (tracked in
git). The harness's personal store
`~/.claude/projects/-home-ib-Desktop-nanobot/memory/` is **symlinked to it** (set up
2026-06-22), so every memory write lands in the repo working tree automatically. After
updating/adding/deleting memory, **`git add .claude/memory && git commit && git push`** as
part of the same change — don't leave memory uncommitted. (`.claude/` is not gitignored.)
