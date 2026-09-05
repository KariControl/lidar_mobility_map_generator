# Releasing

Run this checklist for every public release. A successful command is necessary
evidence, not a substitute for the owner approvals below.

## v0.11.0 release coordinates

The first public release has the following approved identity:

- version: `0.11.0`
- release date: `2026-09-05`
- official release branch: `main`
- release tag: `v0.11.0`

Prepare and review the candidate on `DEVELOP`. After every owner approval and
release-preparation task is complete, create `main` from the reviewed,
publication-safe candidate and rerun the complete release gate below on
`main`. If a file was removed or redacted for privacy, do not branch from
history that still contains the original blob; publish sanitized history
instead. Tag the checked commit as `v0.11.0`. Change the repository visibility
to public only after the gate passes and the tag, branches, documentation, and
public-tree contents have been checked together. These coordinates describe
v0.11.0; update them for a later release while retaining the reusable checklist
below.

## Owner approvals

Before building a release candidate, confirm all of the following:

- Every published image and video, including earlier revisions retained in Git
  history, may be redistributed. Record the owner, source, permission, and any
  required attribution outside the repository when that record is sensitive.
- The complete Git history contains no credential, private location or vehicle
  data, generated artifact, or other material that is not intended to be public.
- A removed or redacted private asset is unreachable from every branch, tag,
  and other ref that will remain in the public repository. Repository visibility
  applies to all published branches, not only the default branch; archive any
  private development history separately before removing it from public refs.
- Every external link works without private credentials and points to content
  intended to remain public.
- The release version and date agree in `package.xml`, `CMakeLists.txt`,
  `CHANGELOG.rst`, and user documentation. Select the release branch and tag
  name, but create the tag only after all checks below pass on the exact commit.
- The repository security settings are ready: private vulnerability reporting,
  a monitored private contact path, secret scanning and push protection where
  available, and branch protection or required checks for the release branch.
- The release claims match the evidence. Vector Map and Navigation Map maturity,
  static validation, simulation, and physical testing must remain distinct.

## Reproducible release gate

Use Ubuntu 24.04 with ROS 2 Jazzy. Start from the committed candidate in its
intended release branch (`main` for public releases) and a clean working tree:

```bash
export LMMG_SOURCE_ROOT="$(git rev-parse --show-toplevel)"
git -C "$LMMG_SOURCE_ROOT" status --short
git -C "$LMMG_SOURCE_ROOT" branch --show-current
git -C "$LMMG_SOURCE_ROOT" rev-parse HEAD
```

`status --short` must print nothing. Audit the reviewed source manifest:

```bash
python3 "$LMMG_SOURCE_ROOT/tools/audit_public_release.py" \
  --mode source "$LMMG_SOURCE_ROOT"
```

Create a new temporary workspace. The public-tree target must not already
exist; the creation tool copies only allowlisted files and audits the result.

```bash
export LMMG_RELEASE_ROOT="$(mktemp -d -t lmmg-release.XXXXXX)"
export LMMG_RELEASE_WS="$LMMG_RELEASE_ROOT/colcon_ws"
export LMMG_PUBLIC_TREE="$LMMG_RELEASE_WS/src/lidar_mobility_map_generator"
mkdir -p "$LMMG_RELEASE_WS/src"
python3 "$LMMG_SOURCE_ROOT/tools/create_public_release_tree.py" \
  "$LMMG_SOURCE_ROOT" "$LMMG_PUBLIC_TREE" \
  --json-output "$LMMG_RELEASE_ROOT/public-tree-audit.json"
python3 "$LMMG_PUBLIC_TREE/tools/audit_public_release.py" \
  --mode tree "$LMMG_PUBLIC_TREE"
```

Check dependencies, then build and test only that fresh public tree:

```bash
source /opt/ros/jazzy/setup.bash
rosdep check --from-paths "$LMMG_PUBLIC_TREE" --ignore-src
cd "$LMMG_RELEASE_WS"
colcon build --symlink-install \
  --packages-select lidar_mobility_map_generator
colcon test --packages-select lidar_mobility_map_generator
colcon test-result --verbose
source "$LMMG_RELEASE_WS/install/setup.bash"
```

If `rosdep check` reports a missing dependency, provision it through the
documented release environment and restart with a new temporary workspace.
Do not accept skipped or failed release-critical tests.

Run the deterministic generated-map demo:

```bash
bash "$LMMG_PUBLIC_TREE/scripts/run_roscon_demo.sh" \
  "$LMMG_RELEASE_ROOT/roscon-demo" generate
```

Run any additional acceptance procedure required by the claims in the release
notes, using reviewed immutable dependencies. Keep its evidence with the
release record; a static check or the deterministic demo does not prove
Autoware/Nav2 runtime behavior or physical deployment readiness.

Finally, verify that no file changed after the candidate was built:

```bash
git -C "$LMMG_SOURCE_ROOT" diff --check
git -C "$LMMG_SOURCE_ROOT" diff --cached --check
git -C "$LMMG_SOURCE_ROOT" diff --exit-code
git -C "$LMMG_SOURCE_ROOT" diff --cached --exit-code
git -C "$LMMG_SOURCE_ROOT" status --short
git -C "$LMMG_SOURCE_ROOT" rev-parse HEAD
git -C "$LMMG_SOURCE_ROOT" branch --show-current
```

The status must still be empty and the commit and branch must match the values
recorded at the start. Review the public-tree audit report and test evidence,
then create the chosen annotated or signed tag on that exact commit and publish
only the audited public content. For v0.11.0, the expected branch and tag are
`main` and `v0.11.0`.
