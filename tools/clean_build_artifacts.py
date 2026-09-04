#!/usr/bin/env python3
"""
Clean build and temporary artifacts before committing the STM32U073 V3.3 project.

Default mode is dry-run. Use --apply to actually delete files/directories.
"""
from __future__ import annotations

import argparse
import fnmatch
import shutil
from pathlib import Path

PROJECT_ROOT = Path(__file__).resolve().parents[1]

DIR_PATTERNS = [
    "MDK-ARM/build",
    "MDK-ARM/bin",
    "MDK-ARM/obj",
    "MDK-ARM/out",
    "MDK-ARM/dist",
    "MDK-ARM/STM32F103_inclinometer  - 4G - V3.3.1",
    "MDK-ARM/STM32U073_inclinometer  - 4G - V3.3.1",
    "MDK-ARM/.eide/log",
    "MDK-ARM/.pack",
    "MDK-ARM/comment_backup_*",
    "MDK-ARM/comment_style_backup_*",
    "MDK-ARM/comment_style_safe_backup_*",
    "**/__pycache__",
]

FILE_PATTERNS = [
    "MDK-ARM/*.uvguix.*",
    "MDK-ARM/*.uvgui.*",
    "MDK-ARM/*.uvgui_*",
    "MDK-ARM/*.lst",
    "MDK-ARM/*.log",
    "MDK-ARM/*.bak",
    "MDK-ARM/.eide.usr.ctx.json",
    "**/*.axf",
    "**/*.elf",
    "**/*.hex",
    "**/*.bin",
    "**/*.s19",
    "**/*.map",
    "**/*.lst",
    "**/*.o",
    "**/*.d",
    "**/*.dep",
    "**/*.crf",
    "**/*.lnp",
    "**/*.htm",
    "**/*.build_log.htm",
    "**/*.scvd",
    "**/*.tmp",
    "**/*.pyc",
]

KEEP_FILES = {
    "MDK-ARM/STM32U073_inclinometer  - 4G - V3.3.1.uvprojx",
    "MDK-ARM/STM32U073_inclinometer  - 4G - V3.3.1.uvoptx",
    "MDK-ARM/STM32U073_inclinometer  - 4G - V3.3.1.code-workspace",
    "MDK-ARM/V33_Config_Tool.spec",
}


def rel(path: Path) -> str:
    return path.relative_to(PROJECT_ROOT).as_posix()


def is_inside_project(path: Path) -> bool:
    try:
        path.resolve().relative_to(PROJECT_ROOT.resolve())
        return True
    except ValueError:
        return False


def glob_paths(patterns: list[str]) -> set[Path]:
    found: set[Path] = set()
    for pattern in patterns:
        if "*" in pattern or "?" in pattern or "[" in pattern:
            found.update(PROJECT_ROOT.glob(pattern))
        else:
            p = PROJECT_ROOT / pattern
            if p.exists():
                found.add(p)
    return found


def should_keep(path: Path) -> bool:
    r = rel(path)
    if r in KEEP_FILES:
        return True
    return any(fnmatch.fnmatch(r, keep) for keep in KEEP_FILES)


def collect_targets() -> tuple[list[Path], list[Path]]:
    dirs = [p for p in glob_paths(DIR_PATTERNS) if p.exists() and p.is_dir()]
    files = [p for p in glob_paths(FILE_PATTERNS) if p.exists() and p.is_file()]

    dirs = [p for p in dirs if is_inside_project(p) and not should_keep(p)]
    files = [p for p in files if is_inside_project(p) and not should_keep(p)]

    # If a file is already inside a directory scheduled for deletion, do not print it twice.
    dir_resolved = [p.resolve() for p in dirs]
    filtered_files = []
    for f in files:
        fr = f.resolve()
        if not any(fr.is_relative_to(d) for d in dir_resolved):
            filtered_files.append(f)

    return sorted(dirs, key=lambda p: rel(p).lower()), sorted(filtered_files, key=lambda p: rel(p).lower())


def main() -> int:
    parser = argparse.ArgumentParser(description="Clean build artifacts for the V3.3 STM32U073 project.")
    parser.add_argument("--apply", action="store_true", help="actually delete files/directories; default is dry-run")
    args = parser.parse_args()

    dirs, files = collect_targets()
    mode = "APPLY" if args.apply else "DRY-RUN"
    print(f"[{mode}] project root: {PROJECT_ROOT}")

    if not dirs and not files:
        print("No build artifacts found.")
        return 0

    if dirs:
        print("\nDirectories:")
        for p in dirs:
            print(f"  {rel(p)}")

    if files:
        print("\nFiles:")
        for p in files:
            print(f"  {rel(p)}")

    if not args.apply:
        print("\nDry-run only. Re-run with --apply to delete these artifacts.")
        return 0

    for p in files:
        p.unlink(missing_ok=True)
    for p in sorted(dirs, key=lambda x: len(x.parts), reverse=True):
        shutil.rmtree(p, ignore_errors=True)

    print("\nClean complete.")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
