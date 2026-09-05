#!/usr/bin/env python3
"""Fail-closed audit for a LiDAR Mobility Map Generator public source tree.

Two audit modes are intentionally separate:

* ``source`` selects only the reviewed public manifest from a development tree.
  Local validation inputs and reports may coexist beside that manifest.
* ``tree`` treats every file below the supplied root as publication content and
  rejects anything outside the manifest.

The release tree, not the mixed development tree, is the authoritative input to
the final pre-publication audit.
"""

from __future__ import annotations

import argparse
import dataclasses
import json
import math
import os
import re
import stat
import struct
import sys
import urllib.parse
import xml.etree.ElementTree as ET
import zlib
from pathlib import Path, PurePosixPath
from typing import Sequence


MAX_PUBLIC_FILE_BYTES = 2 * 1024 * 1024

PUBLIC_TOP_LEVEL_FILES = frozenset(
    {
        ".gitattributes",
        ".gitignore",
        "CHANGELOG.rst",
        "CMakeLists.txt",
        "CONTRIBUTING.md",
        "LICENSE",
        "README.md",
        "README_ja.md",
        "RELEASING.md",
        "SECURITY.md",
        "TRADEMARKS.md",
        "package.xml",
    }
)

PUBLIC_DIRECTORY_FILES: dict[str, frozenset[str]] = {
    ".github": frozenset({"PULL_REQUEST_TEMPLATE.md", "dependabot.yml"}),
    ".github/ISSUE_TEMPLATE": frozenset(
        {"bug_report.yml", "config.yml", "feature_request.yml"}
    ),
    ".github/workflows": frozenset({"ci.yml"}),
    "config": frozenset({"glim.yaml", "review.yaml", "rosbag2.yaml"}),
    "launch": frozenset(
        {
            "edit_and_review.launch.py",
            "edit_navigation_map.launch.py",
            "edit_vector_map.launch.py",
            "generate.launch.py",
            "generate_and_review.launch.py",
            "review.launch.py",
            "semantic_editor.launch.py",
        }
    ),
    "rviz": frozenset({"review.rviz"}),
    "docs": frozenset(
        {
            "operator_manual.md",
            "operator_manual_ja.md",
        }
    ),
    "docs/images": frozenset(
        {
            "autoware_lanelet2_rviz2.png",
            "autoware_vector_map_driving_rviz2.png",
            "logo.png",
            "mid360_input_pointcloud_ja.png",
            "navigation_map_editor_en.png",
            "navigation_map_editor_ja.png",
            "navigation_map_rviz2.png",
            "vector_map_editor_en.png",
            "vector_map_editor_ja.png",
            "vector_map_input_pointcloud_overview.png",
            "vector_map_manual_route_ja.png",
            "vector_map_output_ja.png",
            "vector_map_stop_line_ja.png",
            "vector_map_target_route_ja.png",
        }
    ),
    "docs/videos": frozenset(
        {
            "autoware_vector_map_driving_rviz2.webm",
        }
    ),
    "scripts": frozenset(
        {
            "check_vector_map_edit_smoke.py",
            "check_roscon_demo.py",
            "create_roscon_demo_input.py",
            "follow_nav2_waypoints.py",
            "generation_calibration_contract.py",
            "resolve_autoware_vehicle_info.py",
            "run_nav2_load_only_acceptance.sh",
            "run_roscon_demo.sh",
            "run_vector_map_workflow.sh",
            "stage_autoware_closed_course_map.sh",
            "validate_autoware_candidate.py",
            "verify_target_vehicle_map_binding.py",
        }
    ),
    "test": frozenset(
        {
            "test_autoware_stage.py",
            "test_body_passage_planning.cpp",
            "test_core.cpp",
            "test_evidence_model.cpp",
            "test_follow_nav2_waypoints.py",
            "test_free_space.cpp",
            "test_generation_calibration_contract_direct.py",
            "test_generation_fail_closed.py",
            "test_lanelet_boundary_serialization.cpp",
            "test_nav2_experimental.cpp",
            "test_nav2_load_only_acceptance.py",
            "test_navigation_authoring.cpp",
            "test_observed_route_graph.cpp",
            "test_public_release_tools.py",
            "test_review_launch.py",
            "test_ros_parameters.cpp",
            "test_roscon_demo_fixture.py",
            "test_semantic_route_graph.cpp",
            "test_validate_autoware_candidate.py",
            "test_vector_map_edit_smoke.py",
            "test_vector_map_manual_editor.py",
            "test_vector_map_source.cpp",
        }
    ),
    "tools": frozenset(
        {
            "audit_public_release.py",
            "autoware_lanelet_smoke.cpp",
            "create_public_release_tree.py",
        }
    ),
    "docker/acceptance/locks": frozenset(
        {
            "nav2-load-only-image.lock.env.example",
        }
    ),
    "docker/acceptance/scripts": frozenset(
        {
            "nav2_load_only_probe.py",
            "run_nav2_load_only_acceptance.sh",
            "validate_lock.py",
        }
    ),
}

# These two source directories are part of the public implementation surface.
# Restricting their suffixes prevents an unrelated binary or local note from
# being copied merely because it was placed below a source directory.
PUBLIC_RECURSIVE_SUFFIXES: dict[str, frozenset[str]] = {
    "include/lidar_mobility_map_generator": frozenset({".h", ".hpp"}),
    "src": frozenset({".c", ".cc", ".cpp", ".cxx"}),
}

PUBLIC_EXTRA_FILES = frozenset({"test/data/autoware_local_chain.osm"})

PUBLIC_PNG_FILES = frozenset(
    {
        "docs/images/autoware_lanelet2_rviz2.png",
        "docs/images/autoware_vector_map_driving_rviz2.png",
        "docs/images/logo.png",
        "docs/images/mid360_input_pointcloud_ja.png",
        "docs/images/navigation_map_editor_en.png",
        "docs/images/navigation_map_editor_ja.png",
        "docs/images/navigation_map_rviz2.png",
        "docs/images/vector_map_editor_en.png",
        "docs/images/vector_map_editor_ja.png",
        "docs/images/vector_map_input_pointcloud_overview.png",
        "docs/images/vector_map_manual_route_ja.png",
        "docs/images/vector_map_output_ja.png",
        "docs/images/vector_map_stop_line_ja.png",
        "docs/images/vector_map_target_route_ja.png",
    }
)
PNG_SIGNATURE = b"\x89PNG\r\n\x1a\n"
PUBLIC_WEBM_FILES = frozenset(
    {
        "docs/videos/autoware_vector_map_driving_rviz2.webm",
    }
)
WEBM_SIGNATURE = b"\x1a\x45\xdf\xa3"

RAW_DATA_SUFFIXES = frozenset(
    {".bag", ".db3", ".las", ".laz", ".mcap", ".pcd", ".pgm", ".ply"}
)

FORBIDDEN_COMPONENTS = frozenset(
    {
        ".pytest_cache",
        ".release-local",
        "Testing",
        "__pycache__",
        "build",
        "build_core",
        "generated",
        "install",
        "log",
        "output",
        "release_reports",
        "release_test",
        "user_test_output",
        "user_test_reports",
        "verified_output",
    }
)

LOCAL_ONLY_REFERENCES = frozenset(
    {
        "autoware_1_9_acceptance_behavior_path_planner.param.yaml",
        "autoware_1_9_acceptance_longitudinal_pid.param.yaml",
        "autoware_1_9_acceptance_stop_line.param.yaml",
        "autoware_vehicle_profiles.json",
        "legacy_tuned_autoware_runner.sh",
        "map_ws_fixed_input_lock.json",
        "reference_hesai_yaris_vehicle_info.param.yaml",
        "reference_velodyne_car_vehicle_info.param.yaml",
        "reviewed-runtime-images.lock.env",
        "run_map_ws_dataset.sh",
        "run_map_ws_generation_matrix.sh",
    }
)

LOCAL_REFERENCE_SCAN_EXEMPT = frozenset(
    {
        ".gitignore",
        "test/test_public_release_tools.py",
        "tools/audit_public_release.py",
    }
)

MARKDOWN_LINK_PATTERN = re.compile(r"!?\[[^\]]*\]\(([^)]+)\)")
MARKDOWN_REFERENCE_PATTERN = re.compile(r"(?m)^\s*\[[^\]]+\]:\s*(\S+)")
PERSONAL_PATH_PATTERNS = (
    re.compile(r"/(?:home|Users)/[A-Za-z0-9._-]+(?:/|\b)"),
    re.compile(r"[A-Za-z]:\\Users\\[A-Za-z0-9._-]+(?:\\|\b)"),
)

# GitHub-hosted code is executable supply-chain input. Keep the public workflow
# on a deliberately small allowlist and require immutable revisions verified
# against the maintainers' release commits.
PUBLIC_GITHUB_ACTION_PINS = {
    "actions/checkout": "3d3c42e5aac5ba805825da76410c181273ba90b1",  # v7.0.1
    "ros-tooling/setup-ros": "649ef6bcd696da05bc27ceb3fab69d810c0daeab",  # 0.7.19
}
GITHUB_ACTION_USES_PATTERN = re.compile(
    r"(?m)^[ \t]*-?[ \t]*uses:[ \t]*([^ \t\r\n#]+)"
)
GITHUB_WORKFLOW_FILES = frozenset({".github/workflows/ci.yml"})


@dataclasses.dataclass(frozen=True, order=True)
class AuditIssue:
    """One deterministic public-release audit finding."""

    severity: str
    code: str
    path: str
    message: str

    def as_dict(self) -> dict[str, str]:
        return dataclasses.asdict(self)


@dataclasses.dataclass
class AuditReport:
    """Machine-readable and human-readable audit result."""

    root: str
    mode: str
    files_scanned: int
    bytes_scanned: int
    selected_files: list[str]
    excluded_files: int
    issues: list[AuditIssue]

    @property
    def errors(self) -> list[AuditIssue]:
        return [issue for issue in self.issues if issue.severity == "error"]

    @property
    def warnings(self) -> list[AuditIssue]:
        return [issue for issue in self.issues if issue.severity == "warning"]

    @property
    def ok(self) -> bool:
        return not self.errors

    def as_dict(self) -> dict[str, object]:
        return {
            "schema_version": 1,
            "audit": "lmmg_public_release_surface",
            "root": self.root,
            "mode": self.mode,
            "ok": self.ok,
            "files_scanned": self.files_scanned,
            "bytes_scanned": self.bytes_scanned,
            "selected_files": self.selected_files,
            "excluded_files": self.excluded_files,
            "error_count": len(self.errors),
            "warning_count": len(self.warnings),
            "issues": [issue.as_dict() for issue in self.issues],
        }


def normalize_relative_path(value: str | PurePosixPath) -> str:
    """Return a canonical repository-relative POSIX path or raise."""

    path = PurePosixPath(str(value).replace("\\", "/"))
    if path.is_absolute() or not path.parts or any(part in {"", ".", ".."} for part in path.parts):
        raise ValueError(f"path is not a canonical relative path: {value}")
    return path.as_posix()


def is_public_path_allowed(relative: str | PurePosixPath) -> bool:
    """Return whether a path is selected by the reviewed public manifest."""

    try:
        normalized = normalize_relative_path(relative)
    except ValueError:
        return False
    path = PurePosixPath(normalized)
    if len(path.parts) == 1:
        return normalized in PUBLIC_TOP_LEVEL_FILES
    if normalized in PUBLIC_EXTRA_FILES:
        return True
    for directory, names in PUBLIC_DIRECTORY_FILES.items():
        prefix = PurePosixPath(directory)
        try:
            remainder = path.relative_to(prefix)
        except ValueError:
            continue
        if len(remainder.parts) == 1 and remainder.name in names:
            return True
    for directory, suffixes in PUBLIC_RECURSIVE_SUFFIXES.items():
        prefix = PurePosixPath(directory)
        try:
            remainder = path.relative_to(prefix)
        except ValueError:
            continue
        if remainder.parts and path.suffix.lower() in suffixes:
            return True
    return False


def _could_intersect_public_surface(relative: str) -> bool:
    """Return whether a file or directory path can affect selected content."""

    try:
        normalized = normalize_relative_path(relative)
    except ValueError:
        return False
    if is_public_path_allowed(normalized):
        return True
    exact_paths = set(PUBLIC_TOP_LEVEL_FILES) | set(PUBLIC_EXTRA_FILES)
    for directory, names in PUBLIC_DIRECTORY_FILES.items():
        exact_paths.update(f"{directory}/{name}" for name in names)
    prefix = f"{normalized}/"
    if any(path.startswith(prefix) for path in exact_paths):
        return True
    return any(
        directory == normalized
        or directory.startswith(prefix)
        or normalized.startswith(f"{directory}/")
        for directory in PUBLIC_RECURSIVE_SUFFIXES
    )


def _all_regular_files(root: Path) -> tuple[list[str], list[AuditIssue]]:
    """List regular files without following symlinks."""

    files: list[str] = []
    issues: list[AuditIssue] = []
    for directory, directory_names, file_names in os.walk(root, followlinks=False):
        base = Path(directory)
        kept_directories: list[str] = []
        for name in sorted(directory_names):
            candidate = base / name
            relative = candidate.relative_to(root).as_posix()
            if relative == ".git" or relative.startswith(".git/"):
                continue
            if candidate.is_symlink():
                issues.append(
                    AuditIssue(
                        "error",
                        "symlink",
                        relative,
                        "directory symlinks are not publishable",
                    )
                )
                continue
            kept_directories.append(name)
        directory_names[:] = kept_directories
        for name in sorted(file_names):
            candidate = base / name
            relative = candidate.relative_to(root).as_posix()
            if relative.startswith(".git/"):
                continue
            if candidate.is_symlink():
                issues.append(
                    AuditIssue("error", "symlink", relative, "file symlinks are not publishable")
                )
                continue
            if not candidate.is_file():
                issues.append(
                    AuditIssue(
                        "error",
                        "non_regular_file",
                        relative,
                        "only regular files are publishable",
                    )
                )
                continue
            files.append(relative)
    return sorted(files), issues


def selected_source_files(root: Path) -> tuple[list[str], list[AuditIssue], int]:
    """Select the manifest from a mixed development source tree."""

    all_files, walk_issues = _all_regular_files(root)
    issues = [
        issue for issue in walk_issues if _could_intersect_public_surface(issue.path)
    ]
    selected = sorted(path for path in all_files if is_public_path_allowed(path))
    selected_set = set(selected)

    issues.extend(_missing_manifest_issues(selected_set))
    return selected, issues, len(all_files) - len(selected)


def _missing_manifest_issues(selected: set[str]) -> list[AuditIssue]:
    """Require the complete reviewed surface, not merely a safe subset."""

    issues: list[AuditIssue] = []
    required_exact = set(PUBLIC_TOP_LEVEL_FILES) | set(PUBLIC_EXTRA_FILES)
    for directory, names in PUBLIC_DIRECTORY_FILES.items():
        required_exact.update(f"{directory}/{name}" for name in names)
    for relative in sorted(required_exact - selected):
        issues.append(
            AuditIssue(
                "error",
                "missing_manifest_file",
                relative,
                "required public manifest file is missing",
            )
        )
    for directory in sorted(PUBLIC_RECURSIVE_SUFFIXES):
        if not any(path.startswith(f"{directory}/") for path in selected):
            issues.append(
                AuditIssue(
                    "error",
                    "missing_manifest_directory",
                    directory,
                    "public source directory is empty or missing",
                )
            )
    return issues


def _forbidden_path_reason(relative: str) -> str | None:
    path = PurePosixPath(relative)
    for component in path.parts:
        if component in FORBIDDEN_COMPONENTS:
            return f"forbidden generated/local component: {component}"
        if component.startswith("validation_output") or component.startswith("validation_reports"):
            return f"forbidden validation artifact component: {component}"
        if component.startswith(".lmmg_"):
            return f"forbidden local state component: {component}"
    if path.suffix.lower() in RAW_DATA_SUFFIXES:
        return f"raw or generated mapping data is forbidden: {path.suffix.lower()}"
    return None


def _looks_binary(payload: bytes) -> tuple[bool, str]:
    if payload.startswith(b"\x7fELF"):
        return True, "ELF executable or library"
    known_magic = (
        (b"MZ", "PE executable"),
        (b"PK\x03\x04", "ZIP/archive"),
        (b"%PDF-", "PDF"),
        (b"\x89PNG\r\n\x1a\n", "PNG image"),
        (b"\xff\xd8\xff", "JPEG image"),
        (WEBM_SIGNATURE, "WebM video"),
    )
    for magic, description in known_magic:
        if payload.startswith(magic):
            return True, description
    if b"\x00" in payload:
        return True, "NUL-containing or unknown binary"
    return False, ""


def _invalid_png_reason(payload: bytes) -> str | None:
    """Return a structural PNG error without decoding untrusted pixels."""

    if not payload.startswith(PNG_SIGNATURE):
        return "missing PNG signature"
    offset = len(PNG_SIGNATURE)
    saw_header = False
    saw_data = False
    saw_end = False
    while offset < len(payload):
        if len(payload) - offset < 12:
            return "truncated PNG chunk"
        length = struct.unpack(">I", payload[offset : offset + 4])[0]
        chunk_type = payload[offset + 4 : offset + 8]
        chunk_end = offset + 12 + length
        if chunk_end > len(payload):
            return "PNG chunk length exceeds file"
        if len(chunk_type) != 4 or not all(
            65 <= character <= 90 or 97 <= character <= 122
            for character in chunk_type
        ):
            return "invalid PNG chunk type"
        chunk_data = payload[offset + 8 : offset + 8 + length]
        recorded_crc = struct.unpack(">I", payload[offset + 8 + length : chunk_end])[0]
        computed_crc = zlib.crc32(chunk_data, zlib.crc32(chunk_type)) & 0xFFFFFFFF
        if recorded_crc != computed_crc:
            return "PNG chunk CRC mismatch"
        if not saw_header:
            if chunk_type != b"IHDR" or length != 13:
                return "PNG must start with a 13-byte IHDR"
            width, height = struct.unpack(">II", chunk_data[:8])
            if width == 0 or height == 0 or width > 32768 or height > 32768:
                return "PNG dimensions are invalid or excessive"
            if width * height > 50_000_000:
                return "PNG pixel count exceeds release limit"
            saw_header = True
        elif chunk_type == b"IHDR":
            return "PNG contains multiple IHDR chunks"
        if chunk_type == b"IDAT":
            saw_data = True
        if chunk_type == b"IEND":
            if length != 0:
                return "PNG IEND must be empty"
            saw_end = True
            offset = chunk_end
            if offset != len(payload):
                return "PNG has data after IEND"
            break
        offset = chunk_end
    if not saw_header or not saw_data or not saw_end:
        return "PNG is missing IHDR, IDAT, or IEND"
    return None


def _invalid_webm_reason(payload: bytes) -> str | None:
    """Return a small fail-closed WebM container-header error."""

    if not payload.startswith(WEBM_SIGNATURE):
        return "missing EBML signature"
    header = payload[:4096]
    doc_type = header.find(b"\x42\x82")
    if doc_type < 0 or b"webm" not in header[doc_type : doc_type + 32].lower():
        return "EBML DocType is not webm"
    if b"\x18\x53\x80\x67" not in header:
        return "WebM Segment element is missing"
    return None


def _entropy(value: str) -> float:
    if not value:
        return 0.0
    counts: dict[str, int] = {}
    for character in value:
        counts[character] = counts.get(character, 0) + 1
    length = len(value)
    return -sum((count / length) * math.log2(count / length) for count in counts.values())


def _secret_findings(text: str) -> list[str]:
    findings: list[str] = []
    private_markers = (
        "-----BEGIN " + "PRIVATE KEY-----",
        "-----BEGIN RSA " + "PRIVATE KEY-----",
        "-----BEGIN EC " + "PRIVATE KEY-----",
        "-----BEGIN OPENSSH " + "PRIVATE KEY-----",
    )
    if any(marker in text for marker in private_markers):
        findings.append("private key material")
    token_patterns = (
        re.compile(r"AKIA[0-9A-Z]{16}"),
        re.compile(r"gh[pousr]_[A-Za-z0-9_]{20,}"),
        re.compile(r"github_pat_[A-Za-z0-9_]{20,}"),
        re.compile(r"xox[baprs]-[A-Za-z0-9-]{10,}"),
    )
    if any(pattern.search(text) for pattern in token_patterns):
        findings.append("well-known access-token format")

    assignment = re.compile(
        r"(?i)\b(?:api[_-]?key|access[_-]?token|client[_-]?secret|password)"
        r"\s*[:=]\s*['\"]([A-Za-z0-9_./+=-]{16,})['\"]"
    )
    for match in assignment.finditer(text):
        value = match.group(1)
        classes = sum(
            bool(pattern.search(value))
            for pattern in (
                re.compile(r"[a-z]"),
                re.compile(r"[A-Z]"),
                re.compile(r"[0-9]"),
                re.compile(r"[^A-Za-z0-9]"),
            )
        )
        if classes >= 3 and _entropy(value) >= 3.2:
            findings.append("high-entropy credential assignment")
            break
    return findings


def _extract_local_markdown_target(raw: str) -> str | None:
    value = raw.strip()
    if not value:
        return None
    if value.startswith("<") and ">" in value:
        value = value[1 : value.index(">")]
    else:
        value = value.split(maxsplit=1)[0]
    value = urllib.parse.unquote(value)
    if not value or value.startswith("#"):
        return None
    if re.match(r"^[A-Za-z][A-Za-z0-9+.-]*:", value) or value.startswith("//"):
        return None
    return value.split("#", 1)[0]


def _path_is_within(path: Path, root: Path) -> bool:
    try:
        path.relative_to(root)
    except ValueError:
        return False
    return True


def _audit_markdown_links(
    root: Path,
    relative: str,
    text: str,
    selected_paths: set[str],
) -> list[AuditIssue]:
    issues: list[AuditIssue] = []
    document = root / relative
    raw_targets = MARKDOWN_LINK_PATTERN.findall(text)
    raw_targets.extend(MARKDOWN_REFERENCE_PATTERN.findall(text))
    for raw in raw_targets:
        target = _extract_local_markdown_target(raw)
        if not target:
            continue
        candidate = (document.parent / target).resolve(strict=False)
        if not _path_is_within(candidate, root):
            issues.append(
                AuditIssue(
                    "error",
                    "markdown_link_escape",
                    relative,
                    f"local link escapes release tree: {target}",
                )
            )
        elif not candidate.exists():
            issues.append(
                AuditIssue(
                    "error",
                    "broken_markdown_link",
                    relative,
                    f"local link target is absent: {target}",
                )
            )
        else:
            selected_target = candidate.relative_to(root).as_posix()
            if selected_target not in selected_paths:
                issues.append(
                    AuditIssue(
                        "error",
                        "markdown_link_not_public",
                        relative,
                        f"local link target is excluded from public manifest: {target}",
                    )
                )
    return issues


def _audit_text(
    root: Path,
    relative: str,
    text: str,
    selected_paths: set[str],
) -> list[AuditIssue]:
    issues: list[AuditIssue] = []
    for pattern in PERSONAL_PATH_PATTERNS:
        match = pattern.search(text)
        if match:
            issues.append(
                AuditIssue(
                    "error",
                    "personal_absolute_path",
                    relative,
                    f"personal absolute path found: {match.group(0)}",
                )
            )
            break
    placeholder_email = "maintainer" + "@example.com"
    if placeholder_email in text:
        issues.append(
            AuditIssue(
                "error",
                "placeholder_maintainer",
                relative,
                "placeholder maintainer email must be replaced",
            )
        )
    for finding in _secret_findings(text):
        issues.append(AuditIssue("error", "possible_secret", relative, finding))
    if relative.endswith(".md"):
        issues.extend(_audit_markdown_links(root, relative, text, selected_paths))
    if relative in GITHUB_WORKFLOW_FILES:
        issues.extend(_audit_github_workflow(relative, text))
    if relative not in LOCAL_REFERENCE_SCAN_EXEMPT:
        for local_name in sorted(LOCAL_ONLY_REFERENCES):
            if local_name in text:
                issues.append(
                    AuditIssue(
                        "warning",
                        "excluded_local_reference",
                        relative,
                        f"references excluded local-only file: {local_name}",
                    )
                )
    return issues


def _audit_github_workflow(relative: str, text: str) -> list[AuditIssue]:
    """Reject mutable or unexpectedly privileged GitHub Actions workflows."""

    issues: list[AuditIssue] = []
    observed_actions: set[str] = set()
    for match in GITHUB_ACTION_USES_PATTERN.finditer(text):
        reference = match.group(1).strip("'\"")
        action, separator, revision = reference.rpartition("@")
        expected_revision = PUBLIC_GITHUB_ACTION_PINS.get(action)
        if not separator or expected_revision is None:
            issues.append(
                AuditIssue(
                    "error",
                    "unapproved_github_action",
                    relative,
                    f"GitHub Action is outside the approved allowlist: {reference}",
                )
            )
            continue
        observed_actions.add(action)
        if revision != expected_revision:
            issues.append(
                AuditIssue(
                    "error",
                    "mutable_github_action",
                    relative,
                    f"{action} must use approved full commit SHA {expected_revision}",
                )
            )

    for action in sorted(set(PUBLIC_GITHUB_ACTION_PINS) - observed_actions):
        issues.append(
            AuditIssue(
                "error",
                "missing_github_action",
                relative,
                f"required GitHub Action is absent: {action}",
            )
        )

    if re.search(r"(?m)^[ \t]*pull_request_target[ \t]*:", text):
        issues.append(
            AuditIssue(
                "error",
                "unsafe_github_trigger",
                relative,
                "pull_request_target must not execute the public build workflow",
            )
        )
    permission_declarations = re.findall(r"(?m)^[ \t]*permissions[ \t]*:", text)
    if len(permission_declarations) != 1 or not re.search(
        r"(?m)^permissions:[ \t]*\n  contents:[ \t]*read[ \t]*(?:#.*)?$", text
    ):
        issues.append(
            AuditIssue(
                "error",
                "github_permissions_not_read_only",
                relative,
                "workflow must declare top-level contents: read permissions",
            )
        )
    if re.search(
        r"(?mi)^[ \t]*(?:permissions:[ \t]*write-all|[a-z-]+:[ \t]*write)"
        r"[ \t]*(?:#.*)?$",
        text,
    ):
        issues.append(
            AuditIssue(
                "error",
                "github_write_permission",
                relative,
                "public CI workflow must not request write permissions",
            )
        )
    if not re.search(
        r"(?m)^[ \t]*persist-credentials:[ \t]*false[ \t]*(?:#.*)?$", text
    ):
        issues.append(
            AuditIssue(
                "error",
                "github_credentials_persisted",
                relative,
                "checkout credentials must be disabled after source retrieval",
            )
        )
    return issues


def audit_release_root(root: Path, mode: str = "tree") -> AuditReport:
    """Audit a development source manifest or an already-created release tree."""

    issues: list[AuditIssue] = []
    if mode not in {"source", "tree"}:
        raise ValueError(f"unsupported audit mode: {mode}")
    root_input = root.absolute()
    try:
        root_metadata = os.lstat(root_input)
    except OSError:
        root_metadata = None
    if (
        root_metadata is None
        or stat.S_ISLNK(root_metadata.st_mode)
        or not stat.S_ISDIR(root_metadata.st_mode)
    ):
        issue = AuditIssue(
            "error",
            "invalid_root",
            ".",
            "audit root must be a real directory, not a symlink",
        )
        return AuditReport(str(root_input), mode, 0, 0, [], 0, [issue])
    root = root_input.resolve(strict=True)

    all_files, walk_issues = _all_regular_files(root)
    if mode == "source":
        selected, manifest_issues, excluded_files = selected_source_files(root)
        issues.extend(manifest_issues)
    else:
        issues.extend(walk_issues)
        selected = all_files
        excluded_files = 0
        issues.extend(_missing_manifest_issues(set(selected)))
        for relative in selected:
            if not is_public_path_allowed(relative):
                issues.append(
                    AuditIssue(
                        "error",
                        "unlisted_path",
                        relative,
                        "file is outside the explicit public allowlist",
                    )
                )

    total_bytes = 0
    selected_paths = set(selected)
    for relative in selected:
        candidate = root / relative
        reason = _forbidden_path_reason(relative)
        if reason:
            issues.append(AuditIssue("error", "forbidden_path", relative, reason))
        try:
            size = candidate.stat(follow_symlinks=False).st_size
        except OSError as error:
            issues.append(AuditIssue("error", "stat_failed", relative, str(error)))
            continue
        total_bytes += size
        if size > MAX_PUBLIC_FILE_BYTES:
            issues.append(
                AuditIssue(
                    "error",
                    "oversized_file",
                    relative,
                    f"{size} bytes exceeds {MAX_PUBLIC_FILE_BYTES}-byte public limit",
                )
            )
            continue
        try:
            payload = candidate.read_bytes()
        except OSError as error:
            issues.append(AuditIssue("error", "read_failed", relative, str(error)))
            continue
        binary, binary_description = _looks_binary(payload[:8192])
        if relative in PUBLIC_PNG_FILES:
            if not binary or binary_description != "PNG image":
                issues.append(
                    AuditIssue(
                        "error",
                        "invalid_public_png",
                        relative,
                        "allowlisted screenshot does not have PNG magic",
                    )
                )
                continue
            invalid_png = _invalid_png_reason(payload)
            if invalid_png:
                issues.append(
                    AuditIssue(
                        "error",
                        "invalid_public_png",
                        relative,
                        invalid_png,
                    )
                )
            continue
        if relative in PUBLIC_WEBM_FILES:
            if not binary or binary_description != "WebM video":
                issues.append(
                    AuditIssue(
                        "error",
                        "invalid_public_webm",
                        relative,
                        "allowlisted video does not have WebM magic",
                    )
                )
                continue
            invalid_webm = _invalid_webm_reason(payload)
            if invalid_webm:
                issues.append(
                    AuditIssue(
                        "error",
                        "invalid_public_webm",
                        relative,
                        invalid_webm,
                    )
                )
            continue
        if binary:
            issues.append(
                AuditIssue(
                    "error",
                    "binary_file",
                    relative,
                    f"binary content is not allowlisted: {binary_description}",
                )
            )
            continue
        try:
            text = payload.decode("utf-8")
        except UnicodeDecodeError as error:
            issues.append(AuditIssue("error", "non_utf8_text", relative, str(error)))
            continue
        issues.extend(_audit_text(root, relative, text, selected_paths))
        if PurePosixPath(relative).suffix.lower() in {".osm", ".svg", ".xml"}:
            try:
                ET.fromstring(text)
            except ET.ParseError as error:
                issues.append(AuditIssue("error", "invalid_xml", relative, str(error)))

    license_path = root / "LICENSE"
    if not license_path.is_file() or license_path.is_symlink():
        issues.append(
            AuditIssue(
                "error",
                "missing_license",
                "LICENSE",
                "regular LICENSE file is required",
            )
        )
    else:
        try:
            if not license_path.read_text(encoding="utf-8").strip():
                issues.append(
                    AuditIssue(
                        "error",
                        "empty_license",
                        "LICENSE",
                        "LICENSE must not be empty",
                    )
                )
        except (OSError, UnicodeDecodeError) as error:
            issues.append(AuditIssue("error", "invalid_license", "LICENSE", str(error)))

    deduplicated = sorted(
        set(issues),
        key=lambda item: (item.path, item.severity, item.code, item.message),
    )
    return AuditReport(
        root=str(root),
        mode=mode,
        files_scanned=len(selected),
        bytes_scanned=total_bytes,
        selected_files=selected,
        excluded_files=excluded_files,
        issues=deduplicated,
    )


def render_human(report: AuditReport) -> str:
    """Render a concise deterministic console report."""

    status = "PASS" if report.ok else "FAIL"
    lines = [
        f"PUBLIC_RELEASE_AUDIT={status}",
        f"root={report.root}",
        f"mode={report.mode}",
        f"files_scanned={report.files_scanned}",
        f"bytes_scanned={report.bytes_scanned}",
        f"excluded_files={report.excluded_files}",
        f"errors={len(report.errors)} warnings={len(report.warnings)}",
    ]
    severity_label = {"error": "ERROR", "warning": "WARNING"}
    for issue in report.issues:
        lines.append(
            f"{severity_label.get(issue.severity, issue.severity.upper())} "
            f"[{issue.code}] {issue.path}: {issue.message}"
        )
    return "\n".join(lines)


def _write_json_exclusive(path: Path, document: dict[str, object]) -> None:
    payload = (
        json.dumps(document, ensure_ascii=False, indent=2, sort_keys=True) + "\n"
    ).encode("utf-8")
    flags = os.O_WRONLY | os.O_CREAT | os.O_EXCL
    descriptor = os.open(path, flags, 0o644)
    try:
        with os.fdopen(descriptor, "wb") as stream:
            stream.write(payload)
    except Exception:
        try:
            os.close(descriptor)
        except OSError:
            pass
        raise


def build_argument_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("root", type=Path, nargs="?", default=Path.cwd())
    parser.add_argument("--mode", choices=("source", "tree"), default="tree")
    parser.add_argument("--format", choices=("human", "json"), default="human")
    parser.add_argument(
        "--json-output",
        type=Path,
        help="also write JSON to a new path; an existing file is never overwritten",
    )
    return parser


def main(argv: Sequence[str] | None = None) -> int:
    arguments = build_argument_parser().parse_args(argv)
    report = audit_release_root(arguments.root, arguments.mode)
    document = report.as_dict()
    if arguments.format == "json":
        print(json.dumps(document, ensure_ascii=False, indent=2, sort_keys=True))
    else:
        print(render_human(report))
    if arguments.json_output is not None:
        json_path = arguments.json_output.absolute().resolve(strict=False)
        audited_root = arguments.root.absolute().resolve(strict=False)
        if _path_is_within(json_path, audited_root):
            print("error: JSON report must be outside the audited source/tree", file=sys.stderr)
            return 2
        try:
            _write_json_exclusive(json_path, document)
        except OSError as error:
            print(f"error: cannot write JSON report without overwrite: {error}", file=sys.stderr)
            return 2
    return 0 if report.ok else 1


if __name__ == "__main__":
    raise SystemExit(main())
