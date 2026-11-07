---
name: release
description: Release openlr-decoder to PyPI. Use when asked to release, publish, cut or tag a new version of openlr-decoder.
---

# Release openlr-decoder

Pushing a `vX.Y.Z` tag runs `.github/workflows/release.yml`, which builds the
wheels and sdist and publishes them to PyPI. A PyPI version can't be
re-uploaded, so confirm the version and the tag with the user before pushing
the tag.

The version lives only in `[package] version` of the root `Cargo.toml`.
`pyproject.toml` has `dynamic = ["version"]`, so maturin reads it from there.
The workflow's `check-version` job fails the release if the tag doesn't match.
`crates/road-network` and `crates/map-matcher` have their own versions and are
not published by this workflow.

## Steps

1. Pick the version. List the changes since the last tag with
   `git log --oneline $(git describe --tags --abbrev=0)..main`. Before 1.0,
   bump the minor version for behavior or API changes (different decode
   results, renamed or removed Python API) and the patch version for fixes.
2. On a branch from `main`, set `[package] version` in `Cargo.toml`.
3. Check that the sdist builds on its own, because the package depends on the
   `crates/road-network` path crate. Run `uvx maturin sdist --out <scratch>`,
   install `<scratch>/openlr_decoder-X.Y.Z.tar.gz` into a fresh venv with
   `uv pip install`, and check that `import openlr_decoder` works and that
   `importlib.metadata.version("openlr-decoder")` is `X.Y.Z`.
4. Open a PR titled `Release X.Y.Z` and squash-merge it once CI passes.
   Then do a build-only dry run of the release workflow on `main`:
   `gh workflow run Release --ref main`, find it with
   `gh run list -w Release -e workflow_dispatch -L 1`, and
   `gh run watch --exit-status <id>`. CI doesn't build the macOS wheels, so this
   is the only check that the release runners still work (the `macos-14` image
   was retired between 0.2.7 and 0.3.0). Don't tag until it passes.
5. After confirming with the user, tag the merge commit on `main` and push the
   tag: `git tag vX.Y.Z && git push origin vX.Y.Z`.
6. Find this tag's run with
   `gh run list -w Release --commit $(git rev-parse vX.Y.Z) -L 1 --json databaseId -q '.[0].databaseId'`,
   retrying until it appears, since GitHub takes a few seconds to register it.
   Then watch it with `gh run watch --exit-status <id>`. There is no approval
   step, so an exit status of 0 means the version is on PyPI.
7. Only if step 6 exited 0, create the GitHub release:
   `gh release create vX.Y.Z --generate-notes`.

## If the release run fails

Check PyPI first: `curl -s https://pypi.org/pypi/openlr-decoder/X.Y.Z/json`.
If the version is there, it can't be uploaded again, so fix forward with a
patch release.

If nothing was published, fix the workflow on `main`. Re-running the failed
run won't pick up the fix, because it uses the workflow file at the tagged
commit. The tag has to move to the fix commit, and that is a force-push, which
Claude's permission check always blocks. Give the user the command to run
themselves:

```
! git tag -f vX.Y.Z <fix-commit> && git push -f origin vX.Y.Z
```

The alternative is to bump to the next patch version and tag that. This leaves
the old tag on a commit that was never published.
