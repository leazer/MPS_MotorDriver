#!/usr/bin/env python3
"""Index LKS Scope numeric variables from an ARM ELF's DWARF data."""

from __future__ import annotations

import argparse
import re
import subprocess
from pathlib import Path


DIE_HEADER_RE = re.compile(
    r"^\s*<(?P<depth>\d+)><(?P<offset>[0-9a-fA-F]+)>:"
    r".*?(?:\((?P<tag>DW_TAG_[A-Za-z0-9_]+)\))?\s*$"
)
ATTR_RE = re.compile(r"\bDW_AT_(?P<name>[A-Za-z0-9_]+)\s*:\s*(?P<value>.*)$")
TYPE_REF_RE = re.compile(r"<0x([0-9a-fA-F]+)>")
FORM7_RE = re.compile(
    r"<form\b(?=[^>]*\btype=\"7\")[^>]*>.*?</form>", re.DOTALL
)
VAR_TAG_RE = re.compile(r"<var\b[^>]*/>")
NAME_RE = re.compile(r"\bname=\"([^\"]+)\"")
ADDR_RE = re.compile(r"\baddr=\"[^\"]*\"")


def _attribute_text(value: str) -> str:
    if "): " in value:
        return value.rsplit("): ", 1)[1]
    return value.strip()


def parse_readelf_info(text: str) -> dict[int, dict]:
    """Parse the subset of readelf --debug-dump=info needed for members."""
    dies: dict[int, dict] = {}
    stack: list[tuple[int, int]] = []
    current: dict | None = None

    for line in text.splitlines():
        header = DIE_HEADER_RE.match(line)
        if header:
            depth = int(header.group("depth"))
            while stack and stack[-1][0] >= depth:
                stack.pop()

            tag = header.group("tag")
            current = None
            if tag:
                offset = int(header.group("offset"), 16)
                current = {
                    "offset": offset,
                    "depth": depth,
                    "tag": tag,
                    "attrs": {},
                    "children": [],
                }
                dies[offset] = current
                if stack:
                    dies[stack[-1][1]]["children"].append(offset)
                stack.append((depth, offset))
            continue

        if current is None:
            continue
        attr = ATTR_RE.search(line)
        if attr:
            current["attrs"][attr.group("name")] = attr.group("value").strip()

    return dies


def parse_nm(text: str) -> dict[str, int]:
    symbols: dict[str, int] = {}
    for line in text.splitlines():
        fields = line.split()
        if len(fields) >= 4 and re.fullmatch(r"[0-9a-fA-F]+", fields[0]):
            symbols[fields[3]] = int(fields[0], 16)
        elif len(fields) >= 3 and re.fullmatch(r"[0-9a-fA-F]+", fields[0]):
            symbols[fields[2]] = int(fields[0], 16)
    return symbols


def _type_offset(value: str) -> int:
    match = TYPE_REF_RE.search(value)
    if not match:
        raise ValueError(f"missing DWARF type reference: {value}")
    return int(match.group(1), 16)


def _member_offset(value: str | None) -> int:
    if value is None:
        return 0
    plus_uconst = re.search(r"DW_OP_plus_uconst:\s*(0x[0-9a-fA-F]+|\d+)", value)
    if plus_uconst:
        return int(plus_uconst.group(1), 0)
    number = re.search(r"(?:^|\s)(0x[0-9a-fA-F]+|\d+)(?:\s|$)", value)
    if not number:
        raise ValueError(f"unsupported DWARF member location: {value}")
    return int(number.group(1), 0)


def _unwrap_type(dies: dict[int, dict], offset: int) -> dict:
    wrappers = {
        "DW_TAG_typedef",
        "DW_TAG_const_type",
        "DW_TAG_volatile_type",
        "DW_TAG_restrict_type",
        "DW_TAG_atomic_type",
    }
    seen: set[int] = set()
    while True:
        if offset in seen:
            raise ValueError(f"DWARF type cycle at 0x{offset:x}")
        seen.add(offset)
        die = dies[offset]
        if die["tag"] not in wrappers:
            return die
        offset = _type_offset(die["attrs"]["type"])


def _resolve_members(
    parts: list[str], type_offset: int, dies: dict[int, dict]
) -> int:
    total = 0
    for part in parts:
        aggregate = _unwrap_type(dies, type_offset)
        member = None
        for child_offset in aggregate["children"]:
            child = dies[child_offset]
            if (
                child["tag"] == "DW_TAG_member"
                and _attribute_text(child["attrs"].get("name", "")) == part
            ):
                member = child
                break
        if member is None:
            raise ValueError(
                f"{part!r} is not a member of DWARF type "
                f"0x{aggregate['offset']:x}"
            )
        total += _member_offset(member["attrs"].get("data_member_location"))
        type_offset = _type_offset(member["attrs"]["type"])
    return total


def resolve_expression(
    expression: str, dies: dict[int, dict], symbols: dict[str, int]
) -> int:
    parts = expression.split(".")
    base = parts[0]
    if base not in symbols:
        raise ValueError(f"ELF symbol not found: {base}")
    if len(parts) == 1:
        return symbols[base]

    candidates = [
        die
        for die in dies.values()
        if die["tag"] == "DW_TAG_variable"
        and _attribute_text(die["attrs"].get("name", "")) == base
        and "type" in die["attrs"]
    ]
    failures: list[str] = []
    for variable in candidates:
        try:
            member_offset = _resolve_members(
                parts[1:], _type_offset(variable["attrs"]["type"]), dies
            )
            return symbols[base] + member_offset
        except (KeyError, ValueError) as error:
            failures.append(str(error))

    detail = "; ".join(failures[:3])
    raise ValueError(f"cannot resolve {expression}: {detail or 'no DWARF variable'}")


def _numeric_form(text: str) -> re.Match[str]:
    match = FORM7_RE.search(text)
    if not match:
        raise ValueError("LKS Scope type-7 numeric form not found")
    return match


def numeric_names(text: str) -> list[str]:
    form = _numeric_form(text).group(0)
    names: list[str] = []
    for tag in VAR_TAG_RE.findall(form):
        match = NAME_RE.search(tag)
        if not match:
            raise ValueError(f"numeric var without name: {tag}")
        names.append(match.group(1))
    if len(names) != len(set(names)):
        raise ValueError("duplicate variable names in LKS Scope numeric form")
    return names


def update_scope_text(text: str, addresses: dict[str, int]) -> str:
    form_match = _numeric_form(text)
    form = form_match.group(0)

    def update_tag(match: re.Match[str]) -> str:
        tag = match.group(0)
        name_match = NAME_RE.search(tag)
        if not name_match:
            return tag
        name = name_match.group(1)
        if name not in addresses:
            raise ValueError(f"no indexed address for {name}")
        if not ADDR_RE.search(tag):
            raise ValueError(f"numeric var has no addr attribute: {name}")
        return ADDR_RE.sub(f'addr="0x{addresses[name]:08x}"', tag, count=1)

    updated_form = VAR_TAG_RE.sub(update_tag, form)
    return text[: form_match.start()] + updated_form + text[form_match.end() :]


def _run(command: list[str]) -> str:
    return subprocess.run(
        command,
        check=True,
        stdout=subprocess.PIPE,
        stderr=subprocess.PIPE,
        text=True,
    ).stdout


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "--scope", type=Path, default=Path("debug.lksscope")
    )
    parser.add_argument(
        "--elf", type=Path, default=Path("build/Debug/MPS_MotorDriver.elf")
    )
    parser.add_argument("--readelf", default="arm-none-eabi-readelf")
    parser.add_argument("--nm", default="arm-none-eabi-nm")
    args = parser.parse_args()

    scope_text = args.scope.read_text(encoding="utf-8")
    dwarf_text = _run(
        [args.readelf, "--debug-dump=info", str(args.elf)]
    )
    nm_text = _run(
        [args.nm, "-a", "-S", "--defined-only", str(args.elf)]
    )
    dies = parse_readelf_info(dwarf_text)
    symbols = parse_nm(nm_text)
    names = numeric_names(scope_text)
    addresses = {
        name: resolve_expression(name, dies, symbols) for name in names
    }

    invalid = {
        name: address
        for name, address in addresses.items()
        if not 0x20000000 <= address < 0x20004000
    }
    if invalid:
        details = ", ".join(
            f"{name}=0x{address:08x}" for name, address in invalid.items()
        )
        raise ValueError(f"variables outside target SRAM: {details}")

    args.scope.write_text(
        update_scope_text(scope_text, addresses), encoding="utf-8"
    )
    print(
        f"indexed {len(addresses)} variables from {args.elf} "
        f"into {args.scope}"
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
