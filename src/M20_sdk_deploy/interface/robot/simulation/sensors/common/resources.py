"""Shared M20 sensor resource resolution helpers."""

from __future__ import annotations

from pathlib import Path

try:
    from ament_index_python.packages import get_package_share_directory
except Exception:
    get_package_share_directory = None


PACKAGE_NAME = "m20_sdk_deploy"


def _candidate_package_roots() -> list[Path]:
    roots: list[Path] = []
    seen: set[Path] = set()

    if get_package_share_directory is not None:
        try:
            package_root = Path(get_package_share_directory(PACKAGE_NAME)).resolve()
            roots.append(package_root)
            seen.add(package_root)
        except Exception:
            pass

    for parent in Path(__file__).resolve().parents:
        for candidate in (parent, parent / "share" / PACKAGE_NAME):
            resolved = candidate.resolve()
            if resolved in seen:
                continue
            if (resolved / "M20_description").exists():
                roots.append(resolved)
                seen.add(resolved)

    return roots


def resolve_m20_resource(*parts: str) -> Path:
    rel_path = Path(*parts)
    roots = _candidate_package_roots()

    for root in roots:
        candidate = (root / rel_path).resolve()
        if candidate.exists():
            return candidate

    fallback_root = roots[0] if roots else Path(__file__).resolve().parents[5]
    return (fallback_root / rel_path).resolve()


D435I_XML_PATH = resolve_m20_resource("M20_description", "m20_mjcf", "realsense_d435i", "d435i.xml")
MID360_XML_PATH = resolve_m20_resource("M20_description", "m20_mjcf", "mid360", "mid360.xml")
