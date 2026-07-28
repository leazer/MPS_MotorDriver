from importlib.util import module_from_spec, spec_from_file_location
from pathlib import Path


ROOT = Path(__file__).resolve().parents[2]
SCRIPT = ROOT / "scripts" / "index_lksscope_addresses.py"


def load_indexer():
    spec = spec_from_file_location("index_lksscope_addresses", SCRIPT)
    assert spec is not None and spec.loader is not None
    module = module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def test_nested_member_addresses_follow_dwarf_offsets():
    indexer = load_indexer()
    dwarf = """
 <1><1>: Abbrev Number: 1 (DW_TAG_variable)
    <2>   DW_AT_name        : g_motor_tuning
    <3>   DW_AT_type        : <0x10>
 <1><10>: Abbrev Number: 2 (DW_TAG_structure_type)
 <2><11>: Abbrev Number: 3 (DW_TAG_member)
    <12>   DW_AT_name        : current
    <13>   DW_AT_type        : <0x20>
    <14>   DW_AT_data_member_location: 4
 <1><20>: Abbrev Number: 2 (DW_TAG_structure_type)
 <2><21>: Abbrev Number: 3 (DW_TAG_member)
    <22>   DW_AT_name        : id_kp
    <23>   DW_AT_type        : <0x30>
    <24>   DW_AT_data_member_location: 8
 <1><30>: Abbrev Number: 4 (DW_TAG_base_type)
"""
    nm = "20001000 00000074 B g_motor_tuning\n"

    dies = indexer.parse_readelf_info(dwarf)
    symbols = indexer.parse_nm(nm)

    assert indexer.resolve_expression(
        "g_motor_tuning.current.id_kp", dies, symbols
    ) == 0x2000100C


def test_scope_update_only_changes_numeric_address():
    indexer = load_indexer()
    scope = """<lksscope><forms>
<form type="7"><var name="sample.value" addr=""/></form>
<form type="8"><var name="sample.value" addr="" subPlot="0"/></form>
</forms></lksscope>"""

    updated = indexer.update_scope_text(
        scope, {"sample.value": 0x20001234}
    )

    assert (
        '<form type="7"><var name="sample.value" '
        'addr="0x20001234"/></form>'
    ) in updated
    assert (
        '<form type="8"><var name="sample.value" '
        'addr="" subPlot="0"/></form>'
    ) in updated


if __name__ == "__main__":
    test_nested_member_addresses_follow_dwarf_offsets()
    test_scope_update_only_changes_numeric_address()
    print("lksscope address indexer tests passed")
