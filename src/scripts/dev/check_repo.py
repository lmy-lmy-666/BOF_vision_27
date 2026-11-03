#!/usr/bin/env python3
"""Check that a checkout/archive contains the files needed for redistribution.

Uses only the Python standard library. Does not load ROS or connect to devices.
"""
import argparse
import hashlib
from pathlib import Path
import re
import subprocess
import sys
from urllib.parse import unquote, urlsplit

ROOT = Path(__file__).resolve().parents[3]
ARCHIVE_ENTRIES = (".gitignore", ".gitattributes", ".github", "README.md", "src")


def git_files(*args):
    result = subprocess.check_output(["git", "-C", str(ROOT), "ls-files", "-z", *args])
    return {name.decode("utf-8") for name in result.split(b"\0") if name}


def in_repository():
    try:
        result = subprocess.run(
            ["git", "-C", str(ROOT), "rev-parse", "--show-toplevel"],
            capture_output=True, text=True, check=True,
        )
        return Path(result.stdout.strip()).resolve() == ROOT
    except (FileNotFoundError, subprocess.CalledProcessError):
        return False


def distribution_files():
    if in_repository():
        return {name for name in git_files("--cached", "--others", "--exclude-standard")
                if (ROOT / name).is_file() or (ROOT / name).is_symlink()}
    files = set()
    for name in ARCHIVE_ENTRIES:
        entry = ROOT / name
        for path in entry.rglob("*") if entry.is_dir() else [entry]:
            if (path.is_file() or path.is_symlink()) and "__pycache__" not in path.parts:
                if path.suffix not in (".pyc", ".pyo"):
                    files.add(path.relative_to(ROOT).as_posix())
    return files


def check(files, require_tracked=False):
    errors = []
    allowed = {"src", "build", "install", "log"}
    for entry in ROOT.iterdir():
        if entry.name == "README.md" and entry.is_file():
            continue
        if not entry.name.startswith(".") and (entry.name not in allowed or not entry.is_dir()):
            errors.append(f"Unexpected workspace root entry: {entry.name}")

    def required(path):
        if not path.resolve().is_relative_to(ROOT):
            errors.append(f"External dependency: {path}")
            return
        name = path.relative_to(ROOT).as_posix()
        if not path.is_file() or name not in files:
            errors.append(f"Missing or ignored file: {name}")

    for name in ("README.md", "src/LICENSE", ".gitignore", ".gitattributes",
                 "src/scripts/build.sh", "src/scripts/env.sh", "src/scripts/dev/package.sh",
                 "src/docs/REPOSITORY.md", "src/models/SHA256SUMS", "src/third_party/NOTICE.md"):
        required(ROOT / name)
    for package in ("bof_interfaces", "bof_vision", "bof_bringup"):
        for name in ("CMakeLists.txt", "package.xml"):
            required(ROOT / "src" / package / name)

    manifest = ROOT / "src/models/SHA256SUMS"
    models = set()
    if manifest.is_file():
        for line in manifest.read_text().splitlines():
            if not line.strip():
                continue
            match = re.fullmatch(r"([0-9a-f]{64})  (src/models/[^\s]+)", line)
            if not match:
                errors.append(f"Invalid checksum entry: {line}")
                continue
            digest, name = match.groups()
            path = ROOT / name
            required(path)
            models.add(name)
            if path.is_file() and hashlib.sha256(path.read_bytes()).hexdigest() != digest:
                errors.append(f"Checksum mismatch: {name}")
        if not models:
            errors.append("Model checksum manifest is empty")

    for name in sorted(files):
        path = ROOT / name
        if path.is_symlink():
            target = path.resolve()
            if not target.is_relative_to(ROOT) or not target.exists():
                errors.append(f"External or broken symlink: {name}")
            elif target.is_file() and target.relative_to(ROOT).as_posix() not in files:
                errors.append(f"Symlink target excluded from repository: {name}")
        if path.is_file() and path.stat().st_size >= 100 * 1024 * 1024:
            errors.append(f"File too large for ordinary Git storage: {name}")
        if name.startswith(("build/", "install/", "log/", "artifacts/", "src/data/")):
            errors.append(f"Generated/local data selected for distribution: {name}")
        if name.startswith("src/models/") and path.suffix in (".onnx", ".bin", ".xml"):
            if name not in models:
                errors.append(f"Model missing from checksum manifest: {name}")
        if path.suffix == ".md":
            for link in re.findall(r"\]\(([^\s)]+)\)", path.read_text()):
                url = urlsplit(link)
                if url.scheme or url.netloc or not url.path:
                    continue
                target = (path.parent / unquote(url.path)).resolve()
                required(target)

    # Check the simple scalar resource paths used by the checked-in YAML files.
    pattern = r"^\s*(?:model_path|classifier|extends|geometry_path|calibration_path):\s*([^#\n]*)"
    for path in (ROOT / "src/bof_bringup/config").glob("*.yaml"):
        required(path)
        for value in re.findall(pattern, path.read_text(), re.MULTILINE):
            value = value.strip().strip("\"'")
            if value:
                target = (path.parent / value).resolve()
                required(target)
                if target.suffix == ".xml":
                    required(target.with_suffix(".bin"))

    if require_tracked:
        if not in_repository():
            errors.append("--require-tracked needs a Git checkout")
        else:
            for name in sorted(files - git_files("--cached")):
                errors.append(f"Not staged/tracked: {name}")
    return errors, len(models)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--require-tracked", action="store_true",
                        help="also fail on files that have not been added to Git")
    parser.add_argument("--list-files", action="store_true",
                        help="emit checked distribution paths separated by NUL for tar")
    args = parser.parse_args()
    files = distribution_files()
    errors, count = check(files, args.require_tracked)
    if errors:
        for error in errors:
            print(f"ERROR: {error}", file=sys.stderr)
        return 1
    print(f"Repository OK: {len(files)} files, {count} model assets verified.", file=sys.stderr)
    if args.list_files:
        sys.stdout.buffer.write(b"".join(name.encode("utf-8") + b"\0" for name in sorted(files)))
    return 0


if __name__ == "__main__":
    sys.exit(main())
