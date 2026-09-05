#!/usr/bin/env python3
"""Safety and fail-closed tests for the public-release tools."""

from __future__ import annotations

import contextlib
import base64
import importlib.util
import io
import json
import pathlib
import re
import shutil
import stat
import subprocess
import sys
import tempfile
import unittest


PACKAGE_ROOT = pathlib.Path(__file__).resolve().parents[1]
TINY_VALID_PNG = base64.b64decode(
    "iVBORw0KGgoAAAANSUhEUgAAAAEAAAABCAQAAAC1HAwCAAAAC0lEQVR42mNk"
    "+A8AAQUBAScY42YAAAAASUVORK5CYII="
)
TINY_VALID_WEBM = (
    b"\x1a\x45\xdf\xa3\x8b\x42\x82\x84webm"
    b"\x18\x53\x80\x67\x80"
)


def load_module(name: str, path: pathlib.Path):
    specification = importlib.util.spec_from_file_location(name, path)
    if specification is None or specification.loader is None:
        raise RuntimeError(f"cannot load {path}")
    module = importlib.util.module_from_spec(specification)
    sys.modules[name] = module
    specification.loader.exec_module(module)
    return module


AUDIT = load_module(
    "lmmg_audit_public_release",
    PACKAGE_ROOT / "tools/audit_public_release.py",
)
CREATE = load_module(
    "lmmg_create_public_release_tree",
    PACKAGE_ROOT / "tools/create_public_release_tree.py",
)
GITHUB_WORKFLOW_FIXTURE = f"""name: Public CI fixture
on:
  workflow_dispatch:
permissions:
  contents: read
jobs:
  fixture:
    runs-on: ubuntu-24.04
    steps:
      - uses: actions/checkout@{AUDIT.PUBLIC_GITHUB_ACTION_PINS['actions/checkout']}
        with:
          persist-credentials: false
      - uses: ros-tooling/setup-ros@{AUDIT.PUBLIC_GITHUB_ACTION_PINS['ros-tooling/setup-ros']}
""".encode()


def write_file(root: pathlib.Path, relative: str, content: bytes, executable: bool = False) -> None:
    path = root / relative
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_bytes(content)
    path.chmod(0o755 if executable else 0o644)


def public_fixture(root: pathlib.Path) -> None:
    """Create a small but complete manifest without copying the real package."""

    exact_paths = set(AUDIT.PUBLIC_TOP_LEVEL_FILES) | set(AUDIT.PUBLIC_EXTRA_FILES)
    for directory, names in AUDIT.PUBLIC_DIRECTORY_FILES.items():
        exact_paths.update(f"{directory}/{name}" for name in names)
    for relative in sorted(exact_paths):
        suffix = pathlib.PurePosixPath(relative).suffix.lower()
        if relative == "LICENSE":
            payload = b"Test license\n"
        elif relative in AUDIT.PUBLIC_PNG_FILES:
            payload = TINY_VALID_PNG
        elif relative in AUDIT.PUBLIC_WEBM_FILES:
            payload = TINY_VALID_WEBM
        elif relative in AUDIT.GITHUB_WORKFLOW_FILES:
            payload = GITHUB_WORKFLOW_FIXTURE
        elif relative == "package.xml":
            payload = (
                b"<package><maintainer email='owner@invalid.test'>"
                b"Owner</maintainer></package>\n"
            )
        elif suffix == ".svg":
            payload = b"<svg xmlns='http://www.w3.org/2000/svg'/>\n"
        elif suffix == ".xml":
            payload = b"<root/>\n"
        elif suffix == ".osm":
            payload = b"<osm/>\n"
        else:
            payload = b"public fixture\n"
        executable = suffix in {".py", ".sh"} and (
            relative.startswith("scripts/")
            or relative.startswith("tools/")
            or relative.startswith("docker/acceptance/scripts/")
        )
        write_file(root, relative, payload, executable)
    write_file(root, "include/lidar_mobility_map_generator/fixture.hpp", b"#pragma once\n")
    write_file(root, "src/fixture.cpp", b"int fixture = 0;\n")


class PublicManifestTest(unittest.TestCase):
    @staticmethod
    def markdown_structure(path: pathlib.Path) -> tuple[list[int], list[str], list[str]]:
        heading_levels: list[int] = []
        numbered_sections: list[str] = []
        fence_languages: list[str] = []
        in_fence = False
        for line in path.read_text(encoding="utf-8").splitlines():
            fence = re.fullmatch(r"```([^`]*)", line)
            if fence:
                if not in_fence:
                    fence_languages.append(fence.group(1))
                in_fence = not in_fence
                continue
            if in_fence:
                continue
            heading = re.match(r"^(#{1,6})\s+(.+)$", line)
            if not heading:
                continue
            heading_levels.append(len(heading.group(1)))
            numbered = re.match(r"(\d+(?:\.\d+)?)\.\s", heading.group(2))
            if numbered:
                numbered_sections.append(numbered.group(1))
        if in_fence:
            raise AssertionError(f"unclosed Markdown fence: {path}")
        return heading_levels, numbered_sections, fence_languages

    def test_allowlist_includes_public_surface_and_rejects_private_surface(self) -> None:
        for relative in (
            "src/main.cpp",
            "include/lidar_mobility_map_generator/types.hpp",
            ".github/ISSUE_TEMPLATE/bug_report.yml",
            ".github/ISSUE_TEMPLATE/config.yml",
            ".github/ISSUE_TEMPLATE/feature_request.yml",
            ".github/PULL_REQUEST_TEMPLATE.md",
            ".github/dependabot.yml",
            ".github/workflows/ci.yml",
            "rviz/review.rviz",
            "docs/operator_manual.md",
            "docs/operator_manual_ja.md",
            "docs/images/vector_map_editor_en.png",
            "docs/images/navigation_map_editor_en.png",
            "docs/images/autoware_vector_map_driving_rviz2.png",
            "docs/images/logo.png",
            "docs/images/vector_map_manual_route_ja.png",
            "docs/images/vector_map_input_pointcloud_overview.png",
            "docs/videos/autoware_vector_map_driving_rviz2.webm",
            "tools/audit_public_release.py",
            "tools/autoware_lanelet_smoke.cpp",
            "tools/create_public_release_tree.py",
            "scripts/run_vector_map_workflow.sh",
            "scripts/run_nav2_load_only_acceptance.sh",
            "test/test_vector_map_edit_smoke.py",
            "test/test_nav2_load_only_acceptance.py",
            "test/test_public_release_tools.py",
            "test/test_vector_map_manual_editor.py",
            "test/test_vector_map_source.cpp",
            "docker/acceptance/locks/nav2-load-only-image.lock.env.example",
            "docker/acceptance/scripts/nav2_load_only_probe.py",
            "CONTRIBUTING.md",
            "RELEASING.md",
            "SECURITY.md",
            "TRADEMARKS.md",
        ):
            self.assertTrue(AUDIT.is_public_path_allowed(relative), relative)

        private_paths = (
            "build_core/test_core",
            ".pytest_cache/v/cache/nodeids",
            "config/map_ws_" + "fixed_input_lock.json",
            "config/autoware_1_9_acceptance_" + "longitudinal_pid.param.yaml",
            "docker/acceptance/legacy_" + "tuned_autoware_runner.sh",
            "docker/acceptance/Dockerfile.autoware",
            "docker/acceptance/Dockerfile.nav2",
            "docker/acceptance/compose.yaml",
            "docker/acceptance/locks/runtime-images.lock.env.example",
            "docs/publication_readiness.md",
            "docs/images/unreviewed.png",
            "docs/images/unreviewed.jpeg",
            "launch/autoware_result_review.launch.py",
            "rviz/autoware_result_review.rviz",
            "scripts/autoware_result_replay_model.py",
            "scripts/check_autoware_planning_test_gate.py",
            "scripts/validate_autoware_strict_policy.py",
            "scripts/run_map_" + "ws_dataset.sh",
            "test/test_autoware_speed_limit_runtime_audit.py",
            "validation_output_current/map.pcd",
            "recording.mcap",
        )
        for relative in private_paths:
            self.assertFalse(AUDIT.is_public_path_allowed(relative), relative)

    def test_ci_workflow_is_read_only_pinned_and_runs_the_release_gate(self) -> None:
        workflow = (PACKAGE_ROOT / ".github/workflows/ci.yml").read_text(
            encoding="utf-8"
        )
        self.assertIn("runs-on: ubuntu-24.04", workflow)
        self.assertIn("permissions:\n  contents: read", workflow)
        self.assertIn('LMMG_REQUIRE_WEB_EDITOR_TESTS: "1"', workflow)
        self.assertIn("persist-credentials: false", workflow)
        self.assertNotIn("pull_request_target:", workflow)
        for action, revision in AUDIT.PUBLIC_GITHUB_ACTION_PINS.items():
            self.assertIn(f"uses: {action}@{revision}", workflow)
        for command in (
            "tools/audit_public_release.py --mode source",
            "tools/create_public_release_tree.py",
            '--mode tree "$LMMG_PUBLIC_TREE"',
            "rosdep install --from-paths",
            "rosdep check --from-paths",
            "colcon build",
            "colcon test",
            "colcon test-result --verbose",
            "scripts/run_roscon_demo.sh",
        ):
            self.assertIn(command, workflow)

    def test_github_templates_warn_against_sensitive_map_data(self) -> None:
        templates = (
            PACKAGE_ROOT / ".github/ISSUE_TEMPLATE/bug_report.yml",
            PACKAGE_ROOT / ".github/ISSUE_TEMPLATE/feature_request.yml",
            PACKAGE_ROOT / ".github/PULL_REQUEST_TEMPLATE.md",
        )
        for path in templates:
            text = path.read_text(encoding="utf-8").lower()
            self.assertIn("point cloud", text, path)
            self.assertIn("private", text, path)

    def test_dependabot_reviews_github_actions_updates(self) -> None:
        config = (PACKAGE_ROOT / ".github/dependabot.yml").read_text(
            encoding="utf-8"
        )
        self.assertIn('package-ecosystem: "github-actions"', config)
        self.assertIn('interval: "monthly"', config)

    def test_public_nav2_lock_template_has_no_autoware_entry(self) -> None:
        text = (
            PACKAGE_ROOT
            / "docker/acceptance/locks/nav2-load-only-image.lock.env.example"
        ).read_text(encoding="utf-8")
        self.assertIn("LMMG_NAV2_ACCEPTANCE_IMAGE=", text)
        self.assertNotIn("LMMG_AUTOWARE_ACCEPTANCE_IMAGE", text)

    def test_english_and_japanese_documents_keep_the_same_structure(self) -> None:
        english_manual = self.markdown_structure(PACKAGE_ROOT / "docs/operator_manual.md")
        japanese_manual = self.markdown_structure(PACKAGE_ROOT / "docs/operator_manual_ja.md")
        self.assertEqual(english_manual, japanese_manual)

        english_readme = self.markdown_structure(PACKAGE_ROOT / "README.md")
        japanese_readme = self.markdown_structure(PACKAGE_ROOT / "README_ja.md")
        self.assertEqual(english_readme, japanese_readme)

    def test_tree_audit_passes_complete_text_fixture(self) -> None:
        with tempfile.TemporaryDirectory() as temporary:
            root = pathlib.Path(temporary)
            public_fixture(root)
            report = AUDIT.audit_release_root(root, mode="tree")
            self.assertTrue(report.ok, AUDIT.render_human(report))
            self.assertEqual(report.excluded_files, 0)

    def test_source_mode_selects_manifest_and_ignores_local_products(self) -> None:
        with tempfile.TemporaryDirectory() as temporary:
            root = pathlib.Path(temporary)
            public_fixture(root)
            write_file(root, ".pytest_cache/private", b"not published\n")
            write_file(root, "validation_output_current/map.pcd", b"private\n")
            (root / "validation_output_link").symlink_to(
                root / "validation_output_current", target_is_directory=True
            )
            report = AUDIT.audit_release_root(root, mode="source")
            self.assertTrue(report.ok, AUDIT.render_human(report))
            self.assertGreaterEqual(report.excluded_files, 2)
            self.assertNotIn("validation_output_current/map.pcd", report.selected_files)

    def test_source_link_to_existing_but_excluded_file_fails(self) -> None:
        with tempfile.TemporaryDirectory() as temporary:
            root = pathlib.Path(temporary)
            public_fixture(root)
            write_file(root, "docs/internal.md", b"private development note\n")
            write_file(root, "README.md", b"[internal](docs/internal.md)\n")
            report = AUDIT.audit_release_root(root, mode="source")
            self.assertIn(
                "markdown_link_not_public",
                {issue.code for issue in report.issues},
                AUDIT.render_human(report),
            )
            self.assertNotIn(
                "broken_markdown_link",
                {issue.code for issue in report.issues},
            )

    @unittest.skipUnless(shutil.which("git"), "git is required for ignore-surface test")
    def test_git_add_surface_matches_public_manifest(self) -> None:
        with tempfile.TemporaryDirectory() as temporary:
            root = pathlib.Path(temporary)
            public_fixture(root)
            (root / ".gitignore").write_bytes((PACKAGE_ROOT / ".gitignore").read_bytes())
            write_file(root, "docs/internal.md", b"private\n")
            write_file(root, "scripts/run_autoware_control_test.sh", b"private\n")
            write_file(root, "test/test_internal.py", b"private\n")
            write_file(root, "cmake/local_validation_tests.cmake", b"private\n")
            write_file(
                root,
                "docker/acceptance/locks/runtime-images.lock.env.example",
                b"private\n",
            )
            write_file(root, "recording.mcap", b"private\n")
            subprocess.run(
                ["git", "init", "-q"], cwd=root, check=True, capture_output=True
            )
            result = subprocess.run(
                ["git", "status", "--porcelain=v1", "--untracked-files=all"],
                cwd=root,
                check=True,
                capture_output=True,
                text=True,
            )
            observed = {
                line[3:] for line in result.stdout.splitlines() if line.startswith("?? ")
            }
            report = AUDIT.audit_release_root(root, mode="source")
            self.assertTrue(report.ok, AUDIT.render_human(report))
            self.assertEqual(observed, set(report.selected_files))


class PublicAuditFailureTest(unittest.TestCase):
    def audit_with(self, relative: str, payload: bytes) -> AUDIT.AuditReport:
        temporary = tempfile.TemporaryDirectory()
        self.addCleanup(temporary.cleanup)
        root = pathlib.Path(temporary.name)
        public_fixture(root)
        write_file(root, relative, payload)
        return AUDIT.audit_release_root(root, mode="tree")

    def assert_code(self, report: AUDIT.AuditReport, code: str) -> None:
        self.assertIn(code, {issue.code for issue in report.issues}, AUDIT.render_human(report))

    def test_forbidden_raw_and_unlisted_files_fail(self) -> None:
        report = self.audit_with("recording.mcap", b"private bag\n")
        self.assert_code(report, "unlisted_path")
        self.assert_code(report, "forbidden_path")

    def test_elf_and_unknown_binary_fail(self) -> None:
        report = self.audit_with("src/fixture.cpp", b"\x7fELF" + b"\x00" * 32)
        self.assert_code(report, "binary_file")

    def test_only_allowlisted_structurally_valid_png_files_pass(self) -> None:
        report = self.audit_with("docs/images/vector_map_editor_en.png", b"not png\n")
        self.assert_code(report, "invalid_public_png")

        corrupted = bytearray(TINY_VALID_PNG)
        corrupted[-5] ^= 0x01
        report = self.audit_with(
            "docs/images/vector_map_editor_en.png", bytes(corrupted)
        )
        self.assert_code(report, "invalid_public_png")

        report = self.audit_with("docs/images/extra.png", TINY_VALID_PNG)
        self.assert_code(report, "unlisted_path")
        self.assert_code(report, "binary_file")

        report = self.audit_with("docs/images/extra.jpeg", b"\xff\xd8\xff\x00")
        self.assert_code(report, "unlisted_path")
        self.assert_code(report, "binary_file")

        oversized = TINY_VALID_PNG + b"x" * AUDIT.MAX_PUBLIC_FILE_BYTES
        report = self.audit_with("docs/images/vector_map_editor_en.png", oversized)
        self.assert_code(report, "oversized_file")

    def test_only_allowlisted_webm_with_expected_container_header_passes(self) -> None:
        path = "docs/videos/autoware_vector_map_driving_rviz2.webm"
        report = self.audit_with(path, b"not webm\n")
        self.assert_code(report, "invalid_public_webm")

        corrupted = bytearray(TINY_VALID_WEBM)
        corrupted[0] ^= 0x01
        report = self.audit_with(path, bytes(corrupted))
        self.assert_code(report, "invalid_public_webm")

        report = self.audit_with("docs/videos/extra.webm", TINY_VALID_WEBM)
        self.assert_code(report, "unlisted_path")
        self.assert_code(report, "binary_file")

    def test_oversized_file_fails(self) -> None:
        report = self.audit_with(
            "src/fixture.cpp", b"x" * (AUDIT.MAX_PUBLIC_FILE_BYTES + 1)
        )
        self.assert_code(report, "oversized_file")

    def test_personal_path_placeholder_and_secret_fail(self) -> None:
        personal = "/" + "home" + "/alice/private"
        placeholder = "maintainer" + "@example.com"
        credential = "A1b2C3d4E5f6G7h8I9j0K"
        payload = (
            f"{personal}\n{placeholder}\napi_key = \"{credential}\"\n"
        ).encode()
        report = self.audit_with("README.md", payload)
        codes = {issue.code for issue in report.issues}
        self.assertIn("personal_absolute_path", codes)
        self.assertIn("placeholder_maintainer", codes)
        self.assertIn("possible_secret", codes)

    def test_mutable_unapproved_or_privileged_github_workflow_fails(self) -> None:
        path = ".github/workflows/ci.yml"

        mutable = GITHUB_WORKFLOW_FIXTURE.replace(
            AUDIT.PUBLIC_GITHUB_ACTION_PINS["actions/checkout"].encode(), b"v7"
        )
        self.assert_code(self.audit_with(path, mutable), "mutable_github_action")

        unapproved = GITHUB_WORKFLOW_FIXTURE.replace(
            b"ros-tooling/setup-ros@", b"untrusted/setup-ros@"
        )
        self.assert_code(self.audit_with(path, unapproved), "unapproved_github_action")

        unsafe_trigger = GITHUB_WORKFLOW_FIXTURE.replace(
            b"  workflow_dispatch:\n", b"  pull_request_target:\n"
        )
        self.assert_code(self.audit_with(path, unsafe_trigger), "unsafe_github_trigger")

        writable = GITHUB_WORKFLOW_FIXTURE.replace(b"contents: read", b"contents: write")
        report = self.audit_with(path, writable)
        self.assert_code(report, "github_permissions_not_read_only")
        self.assert_code(report, "github_write_permission")

    def test_broken_markdown_link_and_invalid_svg_fail(self) -> None:
        report = self.audit_with("README.md", b"[missing](does-not-exist.md)\n")
        self.assert_code(report, "broken_markdown_link")
        report = self.audit_with(
            "docs/vector_map_beta_workflow_ja.svg", b"<svg><broken></svg>"
        )
        self.assert_code(report, "invalid_xml")

        report = self.audit_with("README.md", b"[missing-ref]: absent.md\n")
        self.assert_code(report, "broken_markdown_link")

    def test_selected_symlink_fails(self) -> None:
        with tempfile.TemporaryDirectory() as temporary:
            root = pathlib.Path(temporary)
            public_fixture(root)
            (root / "README.md").unlink()
            (root / "README.md").symlink_to("LICENSE")
            report = AUDIT.audit_release_root(root, mode="tree")
            self.assert_code(report, "symlink")

            root_link = root.with_name(root.name + "-link")
            root_link.symlink_to(root, target_is_directory=True)
            self.addCleanup(root_link.unlink, missing_ok=True)
            report = AUDIT.audit_release_root(root_link, mode="tree")
            self.assert_code(report, "invalid_root")

    def test_json_format_is_machine_readable(self) -> None:
        with tempfile.TemporaryDirectory() as temporary:
            root = pathlib.Path(temporary)
            public_fixture(root)
            output = io.StringIO()
            with contextlib.redirect_stdout(output):
                status = AUDIT.main([str(root), "--mode", "tree", "--format", "json"])
            self.assertEqual(status, 0)
            document = json.loads(output.getvalue())
            self.assertTrue(document["ok"])
            self.assertEqual(document["audit"], "lmmg_public_release_surface")

    def test_json_report_is_external_and_never_overwritten(self) -> None:
        with tempfile.TemporaryDirectory() as temporary:
            base = pathlib.Path(temporary)
            root = base / "tree"
            root.mkdir()
            public_fixture(root)
            report_path = base / "audit.json"
            with contextlib.redirect_stdout(io.StringIO()):
                status = AUDIT.main(
                    [str(root), "--mode", "tree", "--json-output", str(report_path)]
                )
            self.assertEqual(status, 0)
            original = report_path.read_bytes()
            with contextlib.redirect_stdout(io.StringIO()), contextlib.redirect_stderr(
                io.StringIO()
            ):
                status = AUDIT.main(
                    [str(root), "--mode", "tree", "--json-output", str(report_path)]
                )
            self.assertEqual(status, 2)
            self.assertEqual(report_path.read_bytes(), original)
            with contextlib.redirect_stdout(io.StringIO()), contextlib.redirect_stderr(
                io.StringIO()
            ):
                status = AUDIT.main(
                    [str(root), "--mode", "tree", "--json-output", str(root / "bad.json")]
                )
            self.assertEqual(status, 2)
            self.assertFalse((root / "bad.json").exists())


class ReleaseTreeCreationTest(unittest.TestCase):
    def test_complete_source_is_copied_and_reaudited_without_git(self) -> None:
        with tempfile.TemporaryDirectory() as temporary:
            base = pathlib.Path(temporary)
            source = base / "source"
            target = base / "release"
            source.mkdir()
            public_fixture(source)
            write_file(source, ".pytest_cache/private", b"not public\n")
            source_report, target_report = CREATE.create_release_tree(source, target)
            self.assertTrue(source_report.ok)
            self.assertTrue(target_report.ok)
            self.assertFalse((target / ".pytest_cache").exists())
            self.assertEqual((target / "src/fixture.cpp").read_bytes(), b"int fixture = 0;\n")
            mode = (target / "tools/audit_public_release.py").stat().st_mode
            self.assertEqual(stat.S_IMODE(mode), 0o755)

    def test_copy_refuses_source_symlink(self) -> None:
        with tempfile.TemporaryDirectory() as temporary:
            base = pathlib.Path(temporary)
            source = base / "source"
            target = base / "target"
            source.mkdir()
            target.mkdir()
            write_file(source, "real.cpp", b"int value;\n")
            (source / "linked.cpp").symlink_to("real.cpp")
            with self.assertRaises(CREATE.ReleaseTreeError):
                CREATE.copy_selected_files(source, target, ["linked.cpp"])
            self.assertEqual(list(target.iterdir()), [])

    def test_existing_nonempty_target_is_unchanged(self) -> None:
        with tempfile.TemporaryDirectory() as temporary:
            base = pathlib.Path(temporary)
            source = base / "source"
            target = base / "target"
            source.mkdir()
            target.mkdir()
            sentinel = target / "keep.txt"
            sentinel.write_text("keep\n", encoding="utf-8")
            with self.assertRaises(CREATE.ReleaseTreeError):
                CREATE._validate_target(source, target)
            self.assertEqual(sentinel.read_text(encoding="utf-8"), "keep\n")

    def test_source_and_target_symlink_roots_are_rejected(self) -> None:
        with tempfile.TemporaryDirectory() as temporary:
            base = pathlib.Path(temporary)
            real_source = base / "real-source"
            real_source.mkdir()
            public_fixture(real_source)
            source_link = base / "source-link"
            source_link.symlink_to(real_source, target_is_directory=True)
            with self.assertRaises(CREATE.ReleaseTreeError):
                CREATE.create_release_tree(source_link, base / "release")

            real_target_parent = base / "real-target-parent"
            real_target_parent.mkdir()
            target_parent_link = base / "target-parent-link"
            target_parent_link.symlink_to(real_target_parent, target_is_directory=True)
            with self.assertRaises(CREATE.ReleaseTreeError):
                CREATE._validate_target(real_source, target_parent_link / "release")
            self.assertEqual(list(real_target_parent.iterdir()), [])

    def test_exclusive_copy_never_overwrites(self) -> None:
        with tempfile.TemporaryDirectory() as temporary:
            base = pathlib.Path(temporary)
            source = base / "source.txt"
            destination = base / "destination.txt"
            source.write_text("new\n", encoding="utf-8")
            destination.write_text("old\n", encoding="utf-8")
            with self.assertRaises(FileExistsError):
                CREATE.copy_file_exclusive(source, destination)
            self.assertEqual(destination.read_text(encoding="utf-8"), "old\n")


if __name__ == "__main__":
    unittest.main()
