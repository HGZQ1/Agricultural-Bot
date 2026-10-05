#!/usr/bin/env python3
"""Compare a generated Xacro model with the audited canonical URDF."""

from __future__ import annotations

import argparse
import subprocess
import sys
import xml.etree.ElementTree as ET
from pathlib import Path


def _load_xml(path: Path) -> ET.Element:
    if path.name.endswith(".xacro"):
        result = subprocess.run(
            ["xacro", str(path)], check=True, capture_output=True, text=True
        )
        return ET.fromstring(result.stdout)
    return ET.parse(path).getroot()


def _fingerprint(element: ET.Element) -> tuple:
    """Return an order-preserving fingerprint for one link or joint subtree."""
    return (
        element.tag,
        tuple(sorted(element.attrib.items())),
        tuple(_fingerprint(child) for child in element),
    )


def _named_elements(root: ET.Element) -> dict[tuple[str, str], tuple]:
    result = {}
    for element in root:
        if element.tag not in {"link", "joint"}:
            continue
        key = (element.tag, element.attrib["name"])
        if key in result:
            raise ValueError(f"duplicate model element: {key}")
        result[key] = _fingerprint(element)
    return result


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("baseline", type=Path)
    parser.add_argument("candidate", type=Path)
    args = parser.parse_args()

    baseline_root = _load_xml(args.baseline)
    candidate_root = _load_xml(args.candidate)
    baseline = _named_elements(baseline_root)
    candidate = _named_elements(candidate_root)

    missing = sorted(baseline.keys() - candidate.keys())
    extra = sorted(candidate.keys() - baseline.keys())
    changed = sorted(
        key for key in baseline.keys() & candidate.keys()
        if baseline[key] != candidate[key]
    )

    if missing or extra or changed:
        if missing:
            print(f"missing: {missing}", file=sys.stderr)
        if extra:
            print(f"extra: {extra}", file=sys.stderr)
        if changed:
            print(f"changed: {changed}", file=sys.stderr)
        return 1

    link_count = sum(tag == "link" for tag, _ in candidate)
    joint_count = sum(tag == "joint" for tag, _ in candidate)
    print(
        f"URDF structures match: {link_count} links, {joint_count} joints; "
        "all origins, axes, limits, inertials, meshes and materials preserved."
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
