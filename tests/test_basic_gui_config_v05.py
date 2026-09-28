"""Static regression checks for V05 configuration tab/field registration.

No GUI display, RoboMaster SDK or hardware required.
"""
import ast
import unittest
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
GUI = ROOT / "classwork8" / "config_gui_v05.py"


def _gui_literals():
    source = ast.parse(GUI.read_text(encoding="utf-8"))
    configure = next(
        node for node in source.body
        if isinstance(node, ast.FunctionDef)
        and node.name == "configure_before_run"
    )
    values = {}
    for node in configure.body:
        if isinstance(node, (ast.Assign, ast.AnnAssign)):
            targets = node.targets if isinstance(node, ast.Assign) else [node.target]
            for target in targets:
                if isinstance(target, ast.Name) and target.id in ("tab_names", "field_specs"):
                    values[target.id] = ast.literal_eval(node.value)
    return values["tab_names"], values["field_specs"]


class GuiRegistrationTests(unittest.TestCase):
    def test_every_field_spec_tab_is_registered(self):
        tab_names, field_specs = _gui_literals()
        self.assertEqual(set(tab_names), set(field_specs))
        self.assertEqual(len(tab_names), len(set(tab_names)))

    def test_basic_motion_tab_uses_mapping_name(self):
        tab_names, field_specs = _gui_literals()
        self.assertIn("ToF / Mapping", tab_names)
        self.assertIn("ToF / Mapping", field_specs)

    def test_live_map_and_png_use_same_visible_target_filter(self):
        gui = (ROOT / "classwork8" / "gui_v05.py").read_text(encoding="utf-8")
        # Both map viewport calculations and both drawing passes use the
        # same filter. LOS should not alter view bounds or appear in exports.
        self.assertEqual(
            gui.count('for target in visible_map_targets(snapshot.get("targets") or []):'),
            4,
        )
        self.assertNotIn('"? LOS"', gui)
        self.assertNotIn('"Hollow Txx? LOS', gui)
        self.assertIn("Targets on map:", gui)

    def test_no_obsolete_speed_cap_controls_are_shown(self):
        _tab_names, field_specs = _gui_literals()
        visible = {
            attr
            for specs in field_specs.values()
            for attr, _label, _kind, _help in specs
        }
        self.assertNotIn("motion_wall_adjacent_speed_cap_mps", visible)
        self.assertNotIn("motion_slow_cross_track_speed_mps", visible)
        self.assertIn("travel_speed_mps", visible)


if __name__ == "__main__":
    unittest.main()
