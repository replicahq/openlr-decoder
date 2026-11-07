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
5. After confirming with the user, tag the merge commit on `main` and push the
   tag: `git tag vX.Y.Z && git push origin vX.Y.Z`.
6. Watch the run with `gh run watch $(gh run list -w Release -L 1 --json databaseId -q '.[0].databaseId')`.
   There is no approval step, so a passing run is on PyPI.
7. After it publishes, create the GitHub release:
   `gh release create vX.Y.Z --generate-notes`.
