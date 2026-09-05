#!/usr/bin/env python3
"""Create a new public source tree from the reviewed explicit allowlist.

The command never uses Git state, never follows symlinks, and never overwrites
or removes a target entry.  The source manifest is audited before the first
target directory is created and the completed tree is audited again.
"""

from __future__ import annotations

import argparse
import json
import os
import stat
import sys
from pathlib import Path
from typing import Sequence


SCRIPT_DIRECTORY = Path(__file__).resolve().parent
if str(SCRIPT_DIRECTORY) not in sys.path:
    sys.path.insert(0, str(SCRIPT_DIRECTORY))

from audit_public_release import (  # noqa: E402
    AuditReport,
    audit_release_root,
    render_human,
)


class ReleaseTreeError(RuntimeError):
    """A fail-closed release-tree construction error."""


def _path_is_within(path: Path, root: Path) -> bool:
    try:
        path.relative_to(root)
    except ValueError:
        return False
    return True


def _validate_source_file(source_root: Path, relative: str) -> Path:
    """Resolve no links and require every selected component to be real."""

    current = source_root
    for part in relative.split("/"):
        current = current / part
        try:
            metadata = os.lstat(current)
        except OSError as error:
            raise ReleaseTreeError(f"cannot inspect selected source {relative}: {error}") from error
        if stat.S_ISLNK(metadata.st_mode):
            raise ReleaseTreeError(f"selected source is or traverses a symlink: {relative}")
    if not stat.S_ISREG(os.lstat(current).st_mode):
        raise ReleaseTreeError(f"selected source is not a regular file: {relative}")
    return current


def _validate_target(source_root: Path, target: Path) -> tuple[Path, bool]:
    """Require a new or empty target while preserving every existing entry."""

    source_root = source_root.resolve(strict=True)
    target_absolute = target.absolute()
    current = Path(target_absolute.anchor)
    for part in target_absolute.parts[1:-1]:
        current = current / part
        if not (current.exists() or current.is_symlink()):
            break
        metadata = os.lstat(current)
        if stat.S_ISLNK(metadata.st_mode):
            raise ReleaseTreeError(f"target path traverses a symlink: {current}")
    target_resolved = target_absolute.resolve(strict=False)
    if target_resolved == source_root:
        raise ReleaseTreeError("source and target directories must differ")
    if _path_is_within(source_root, target_resolved):
        raise ReleaseTreeError("target must not contain the source tree")
    if _path_is_within(target_resolved, source_root):
        relative_target = target_resolved.relative_to(source_root)
        if not relative_target.parts or relative_target.parts[0] != ".release-local":
            raise ReleaseTreeError(
                "a target inside the source is allowed only below .release-local/"
            )

    existed = target_absolute.exists() or target_absolute.is_symlink()
    if existed:
        metadata = os.lstat(target_absolute)
        if stat.S_ISLNK(metadata.st_mode):
            raise ReleaseTreeError("target directory must not be a symlink")
        if not stat.S_ISDIR(metadata.st_mode):
            raise ReleaseTreeError("target exists and is not a directory")
        try:
            next(target_absolute.iterdir())
        except StopIteration:
            pass
        else:
            raise ReleaseTreeError("target directory is not empty; nothing was changed")
    return target_absolute, existed


def _mkdir_without_symlink(path: Path) -> None:
    if path.exists() or path.is_symlink():
        metadata = os.lstat(path)
        if stat.S_ISLNK(metadata.st_mode) or not stat.S_ISDIR(metadata.st_mode):
            raise ReleaseTreeError(f"target parent is not a real directory: {path}")
        return
    path.mkdir()
    metadata = os.lstat(path)
    if stat.S_ISLNK(metadata.st_mode) or not stat.S_ISDIR(metadata.st_mode):
        raise ReleaseTreeError(f"target directory was replaced unexpectedly: {path}")


def _create_target_parents(target_root: Path, relative: str) -> Path:
    destination = target_root / relative
    current = target_root
    for part in relative.split("/")[:-1]:
        current = current / part
        _mkdir_without_symlink(current)
    return destination


def copy_file_exclusive(source: Path, destination: Path) -> None:
    """Copy one regular file without following links or replacing a path."""

    source_flags = os.O_RDONLY | getattr(os, "O_NOFOLLOW", 0)
    source_descriptor = os.open(source, source_flags)
    try:
        source_metadata = os.fstat(source_descriptor)
        if not stat.S_ISREG(source_metadata.st_mode):
            raise ReleaseTreeError(f"source is not a real regular file: {source}")
        destination_descriptor = os.open(
            destination, os.O_WRONLY | os.O_CREAT | os.O_EXCL, 0o600
        )
        with os.fdopen(source_descriptor, "rb") as input_stream, os.fdopen(
            destination_descriptor, "wb"
        ) as output_stream:
            source_descriptor = -1
            while True:
                block = input_stream.read(1024 * 1024)
                if not block:
                    break
                output_stream.write(block)
    finally:
        if source_descriptor >= 0:
            os.close(source_descriptor)
    public_mode = 0o755 if source_metadata.st_mode & 0o111 else 0o644
    os.chmod(destination, public_mode, follow_symlinks=False)


def copy_selected_files(
    source_root: Path, target_root: Path, relative_paths: Sequence[str]
) -> None:
    """Copy an already-reviewed list; primarily useful to test safety rules."""

    _mkdir_without_symlink(target_root)
    for relative in sorted(relative_paths):
        source = _validate_source_file(source_root, relative)
        destination = _create_target_parents(target_root, relative)
        if destination.exists() or destination.is_symlink():
            raise ReleaseTreeError(f"refusing to overwrite target entry: {relative}")
        copy_file_exclusive(source, destination)


def create_release_tree(source_root: Path, target: Path) -> tuple[AuditReport, AuditReport]:
    """Audit, exclusively copy, and re-audit a public release tree."""

    source_input = source_root.absolute()
    source_metadata = os.lstat(source_input)
    if stat.S_ISLNK(source_metadata.st_mode) or not stat.S_ISDIR(source_metadata.st_mode):
        raise ReleaseTreeError("source root must be a real directory")
    source_root = source_input.resolve(strict=True)
    source_report = audit_release_root(source_root, mode="source")
    if not source_report.ok:
        raise ReleaseTreeError(
            "source public manifest failed audit before target creation\n"
            + render_human(source_report)
        )
    selected = source_report.selected_files
    target_root, target_existed = _validate_target(source_root, target)

    # Validate every source component before creating the target.  This keeps a
    # bad manifest or source symlink from leaving a partial release directory.
    for relative in selected:
        _validate_source_file(source_root, relative)

    if not target_existed:
        target_root.parent.mkdir(parents=True, exist_ok=True)
        _mkdir_without_symlink(target_root)
    copy_selected_files(source_root, target_root, selected)
    tree_report = audit_release_root(target_root, mode="tree")
    if not tree_report.ok:
        raise ReleaseTreeError(
            "copied release tree failed final audit; it was left intact for inspection\n"
            + render_human(tree_report)
        )
    return source_report, tree_report


def _write_json_exclusive(path: Path, document: dict[str, object]) -> None:
    payload = (
        json.dumps(document, ensure_ascii=False, indent=2, sort_keys=True) + "\n"
    ).encode("utf-8")
    descriptor = os.open(path, os.O_WRONLY | os.O_CREAT | os.O_EXCL, 0o644)
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
    parser.add_argument("source", type=Path)
    parser.add_argument("target", type=Path)
    parser.add_argument(
        "--json-output",
        type=Path,
        help="write a creation/audit report to a new file outside the release tree",
    )
    return parser


def main(argv: Sequence[str] | None = None) -> int:
    arguments = build_argument_parser().parse_args(argv)
    try:
        source_report, tree_report = create_release_tree(arguments.source, arguments.target)
    except (OSError, ReleaseTreeError) as error:
        print(f"error: {error}", file=sys.stderr)
        return 1

    document: dict[str, object] = {
        "schema_version": 1,
        "operation": "lmmg_create_public_release_tree",
        "source": source_report.as_dict(),
        "release_tree": tree_report.as_dict(),
    }
    print(render_human(tree_report))
    print(f"PUBLIC_RELEASE_TREE={Path(arguments.target).resolve(strict=True)}")
    if arguments.json_output is not None:
        json_path = arguments.json_output.absolute().resolve(strict=False)
        target_root = arguments.target.absolute().resolve(strict=True)
        if _path_is_within(json_path, target_root):
            print("error: JSON report must be outside the audited release tree", file=sys.stderr)
            return 2
        try:
            _write_json_exclusive(json_path, document)
        except OSError as error:
            print(f"error: cannot write JSON report without overwrite: {error}", file=sys.stderr)
            return 2
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
